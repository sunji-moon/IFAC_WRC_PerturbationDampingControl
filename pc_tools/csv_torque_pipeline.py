"""
csv_torque_pipeline.py

Offline "expected torque profile" pipeline for the detector-triggered hip
damping controller in
    XM_Firmware/XM_Apps/User_Algorithm/damping_assist_control.c

It answers: "If I flashed damping_assist_control.c as user_app.c and a recorded
perturbation had happened while it was ACTIVE and armed, what torque would the
H10 have commanded?" — computed from data we already have (the 0821 ND_5 robot
CSVs), before ever putting torque on a person.

WHY A SEPARATE ENGINE (not the GUI's simulate_controller)
---------------------------------------------------------
The tuning GUI's simulate_controller is a quick vectorized preview that assumes
the perturbation onset is at t=0 and uses np.gradient velocity. The real firmware
is a *sample-stepped state machine*:

    - hip angular velocity from the ENCODER angle via change-detection + 15 Hz
      LPF  (_UpdateHipVelocity in the .c),
    - WAIT / ACTIVE / REFRACTORY states,
    - event-specific enable rules (which leg, flexion vs extension),
    - stop when the primary hip velocity REVERSES (not a fixed window),
    - tau = torque_sign * (-B_eff * qdot_rad), with optional clamp OFF by default.

This file ports that state machine faithfully so the plotted torque matches what
the MCU would actually output. Tuning coefficients are shared with the GUI via
ControllerParams, so the two stay in lock-step.

WHAT IT NEEDS FROM THE CSV
--------------------------
Real robot CSV columns used:  mcu_time_ms, right_hip_angle_deg, left_hip_angle_deg,
h10_right_foot_contact, h10_left_foot_contact  (all present in the ND_5 logs).
Velocity is RECONSTRUCTED from the encoder angle exactly like the firmware; the
logged *_hip_velocity_dps column (a 20 Hz DAQ estimate) is only shown for
reference.

CAVEATS (be honest)
-------------------
* These trials were recorded in transparent/monitor mode (torque approx 0), so
  this shows the *open-loop* torque the controller would have requested against
  the recorded motion. It does NOT show how the leg would then have responded
  (no closed loop). It is for shaping / sanity-checking coefficients and signs.
* Onset timing + type + perturbed side are read from the trial's sibling
  *_events.csv (perturbation_onset rows: perturbation_event_id/type/
  affected_leg/latest_mcu_time_ms — schema_version 2 logs carry exactly 4 per
  trial). perturbation_invalid / perturbation_recovery rows are ignored. Every
  labelled onset in a trial is simulated in sequence, gated by the same
  ACTIVE + REFRACTORY IDLE-check as DampingAssist_Trigger() in the .c (an
  onset that lands before the previous reaction's refractory has expired is
  skipped, exactly like real firmware would ignore it). Pass --no-events to
  fall back to the old heuristic (largest hip-velocity spike in the trial;
  --onset/--side override) for logs that predate the events.csv schema.
* h10_*_foot_contact is an H10 estimate, not a foot switch. It is only used for
  trip ACTIVE timing and can be replaced by --trip-delay.

USAGE
-----
Single trial, show + save:
    python csv_torque_pipeline.py "data/20260821/ND_5/..._trip_like_..._T03_robot.csv" \
        --event TRIP --show --save out.png

Batch every ND_5 trial to PNGs:
    python csv_torque_pipeline.py --batch "data/20260821/ND_5" --outdir torque_png

Drive the tuning GUI on a real trial (sliders update the real-CSV torque):
    python csv_torque_pipeline.py "..._T03_robot.csv" --event TRIP --gui

Dependencies: numpy, pandas, matplotlib (+ tkinter for --gui).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

# Share the tuning parameter object with the GUI so coefficients stay in one
# place. If the GUI module cannot be imported (e.g. headless dep issue) we fall
# back to an equivalent dataclass carrying the .c defaults.
try:
    from damping_tuning_gui import ControllerParams as _GuiParams
    _HAVE_GUI_PARAMS = True
except Exception:  # pragma: no cover - defensive
    _GuiParams = None
    _HAVE_GUI_PARAMS = False


DEG2RAD = math.pi / 180.0
TWO_PI = 2.0 * math.pi

# Firmware constants (must match damping_assist_control.c section 5).
ANGLE_EPS_DEG = 0.005
VEL_STALE_TIMEOUT_MS = 120
VEL_MAX_GAP_MS = 60

# Perturbation type codes mirror the C enum PerturbationType_t.
PERT_NONE = 0
PERT_ES_SLIP = 1
PERT_LS_SLIP = 2
PERT_TRIP = 3

EVENT_CODE_TO_NAME = {
    PERT_ES_SLIP: "ES slip",
    PERT_LS_SLIP: "LS slip",
    PERT_TRIP: "Early-swing trip",
}

LEG_RIGHT = 1
LEG_LEFT = 2

# events.csv perturbation_type -> firmware PerturbationType_t. heel_slip hits
# early stance (impact leg flexes to catch it) -> ES; toe_slip hits late
# stance / push-off (slipping leg extends) -> LS. Mirrors _StartReaction()'s
# case PERT_ES_SLIP / PERT_LS_SLIP comments in the .c.
EVENT_TYPE_STR_TO_CODE = {
    "heel_slip": PERT_ES_SLIP,
    "toe_slip": PERT_LS_SLIP,
    "trip": PERT_TRIP,
}
SIDE_STR_TO_CODE = {"right": LEG_RIGHT, "left": LEG_LEFT}


# --------------------------------------------------------------------------- #
# Tuning parameters                                                           #
# --------------------------------------------------------------------------- #
@dataclass
class FirmwareParams:
    """Superset of the GUI's ControllerParams + the extra firmware-only knobs
    (min-active, refractory, velocity LPF) that the vectorized GUI omits.
    Defaults mirror damping_assist_control.c section 2."""

    # damping coefficients [Nm*s/rad]
    k_es_impact: float = 0.30
    k_es_stance: float = 0.30
    k_ls_slip: float = 0.30
    k_ls_stance: float = 0.30
    k_trip_impact: float = 0.30
    k_trip_stance: float = 0.30

    # scaling
    angle_gain_per_deg: float = 0.02
    trip_com_scaling_enable: bool = False
    trip_com_gain_per_mps: float = 1.0
    manual_mean_com_velocity_mps: float = 0.8

    # sign conventions
    right_flexion_sign: int = +1
    left_flexion_sign: int = +1
    right_torque_sign: int = +1
    left_torque_sign: int = +1

    # output clamp (OFF by default, matching the .c request)
    enable_torque_clamp: bool = False
    torque_limit_nm: float = 10.0

    # timing / stop
    # NOTE: manual-detector bench config (matches damping_assist_control.c) --
    # _run_reaction()'s stop rule is pure sign-change (discrete IVT) on the
    # primary leg's qdot, so zero_cross_threshold_dps and max_active_ms below
    # are NOT read there anymore (still used by the synthetic-preview GUI's
    # simulate_controller() in damping_tuning_gui.py, which is unchanged).
    zero_cross_threshold_dps: float = 5.0
    min_active_before_stop_ms: int = 20
    max_active_ms: int = 700
    refractory_ms: int = 250
    trip_fallback_delay_ms: int = 220

    # velocity estimator
    vel_lpf_hz: float = 15.0

    @classmethod
    def from_gui_params(cls, gp) -> "FirmwareParams":
        """Build from the GUI's ControllerParams, keeping firmware-only defaults."""
        return cls(
            k_es_impact=gp.k_es_impact,
            k_es_stance=gp.k_es_stance,
            k_ls_slip=gp.k_ls_slip,
            k_ls_stance=gp.k_ls_stance,
            k_trip_impact=gp.k_trip_impact,
            k_trip_stance=gp.k_trip_stance,
            angle_gain_per_deg=gp.angle_gain_per_deg,
            trip_com_scaling_enable=gp.trip_com_scaling_enable,
            trip_com_gain_per_mps=gp.trip_com_gain_per_mps,
            manual_mean_com_velocity_mps=gp.manual_mean_com_velocity_mps,
            right_flexion_sign=gp.right_flexion_sign,
            left_flexion_sign=gp.left_flexion_sign,
            right_torque_sign=gp.right_torque_sign,
            left_torque_sign=gp.left_torque_sign,
            enable_torque_clamp=gp.enable_torque_clamp,
            torque_limit_nm=gp.torque_limit_nm,
            zero_cross_threshold_dps=gp.zero_cross_threshold_dps,
            max_active_ms=int(gp.max_active_ms),
            trip_fallback_delay_ms=int(gp.trip_foot_contact_delay_ms),
        )


# --------------------------------------------------------------------------- #
# Robot CSV loading                                                           #
# --------------------------------------------------------------------------- #
@dataclass
class TrialData:
    path: str
    time_s: np.ndarray          # seconds, zero-based
    time_ms: np.ndarray         # int ms, the firmware clock (zero-based)
    time_ms_origin: float       # the raw mcu_time_ms of sample 0, before zeroing
                                 # (subtract this from an events.csv
                                 # latest_mcu_time_ms to land it on time_ms)
    rh_angle: np.ndarray        # deg
    lh_angle: np.ndarray        # deg
    rh_foot: np.ndarray         # bool
    lh_foot: np.ndarray         # bool
    rh_vel_logged: np.ndarray | None = None  # DAQ 20 Hz estimate, reference only
    lh_vel_logged: np.ndarray | None = None


def _col(df: pd.DataFrame, *names):
    lower = {str(c).lower(): c for c in df.columns}
    for n in names:
        if n.lower() in lower:
            return lower[n.lower()]
    return None


def load_robot_csv(path: str) -> TrialData:
    df = pd.read_csv(path)

    t_col = _col(df, "mcu_time_ms", "time_ms", "time_s", "time", "t")
    rh_col = _col(df, "right_hip_angle_deg", "rightHipAngle", "rh_angle")
    lh_col = _col(df, "left_hip_angle_deg", "leftHipAngle", "lh_angle")
    if rh_col is None or lh_col is None:
        raise ValueError(
            f"{path}: needs right/left hip angle columns "
            "(right_hip_angle_deg, left_hip_angle_deg)."
        )

    if t_col is None:
        time_ms = np.arange(len(df), dtype=float) * 10.0  # assume 100 Hz
    else:
        t = pd.to_numeric(df[t_col], errors="coerce").to_numpy(dtype=float)
        if t_col.lower() in ("time_s", "time", "t"):
            t = t * 1000.0  # seconds -> ms
        time_ms = t
    time_ms_origin = float(time_ms[0])
    time_ms = time_ms - time_ms[0]

    def clean(c):
        return (
            pd.to_numeric(df[c], errors="coerce")
            .interpolate()
            .bfill()
            .ffill()
            .to_numpy(dtype=float)
        )

    rh_angle = clean(rh_col)
    lh_angle = clean(lh_col)

    rh_foot_col = _col(df, "h10_right_foot_contact", "isRightFootContact")
    lh_foot_col = _col(df, "h10_left_foot_contact", "isLeftFootContact")
    rh_foot = (clean(rh_foot_col) > 0.5) if rh_foot_col else np.zeros(len(df), bool)
    lh_foot = (clean(lh_foot_col) > 0.5) if lh_foot_col else np.zeros(len(df), bool)

    rhv_col = _col(df, "right_hip_velocity_dps", "right_hip_vel_dps")
    lhv_col = _col(df, "left_hip_velocity_dps", "left_hip_vel_dps")

    return TrialData(
        path=path,
        time_s=time_ms / 1000.0,
        time_ms=np.round(time_ms).astype(np.int64),
        time_ms_origin=time_ms_origin,
        rh_angle=rh_angle,
        lh_angle=lh_angle,
        rh_foot=rh_foot,
        lh_foot=lh_foot,
        rh_vel_logged=clean(rhv_col) if rhv_col else None,
        lh_vel_logged=clean(lhv_col) if lhv_col else None,
    )


# --------------------------------------------------------------------------- #
# Firmware-faithful velocity estimator (port of _UpdateHipVelocity)           #
# --------------------------------------------------------------------------- #
def _lpf_alpha(cutoff_hz: float, dt_s: float) -> float:
    if cutoff_hz <= 0.0:
        return 1.0
    wc_dt = TWO_PI * cutoff_hz * dt_s
    return wc_dt / (1.0 + wc_dt)


def firmware_velocity(time_ms: np.ndarray, angle_deg: np.ndarray,
                      lpf_hz: float) -> np.ndarray:
    """Exact port of _UpdateHipVelocity() run continuously over the trial."""
    n = len(angle_deg)
    vel = np.zeros(n, dtype=float)
    initialized = False
    last_angle = 0.0
    last_change_tick = 0
    v = 0.0

    for i in range(n):
        now = int(time_ms[i])
        a = float(angle_deg[i])

        if not initialized:
            initialized = True
            last_angle = a
            last_change_tick = now
            v = 0.0
            vel[i] = 0.0
            continue

        d_angle = a - last_angle

        if abs(d_angle) < ANGLE_EPS_DEG:
            if (now - last_change_tick) > VEL_STALE_TIMEOUT_MS:
                v = 0.0
            vel[i] = v
            continue

        elapsed_ms = now - last_change_tick
        if elapsed_ms == 0:
            vel[i] = v
            continue
        if elapsed_ms > VEL_MAX_GAP_MS:
            last_angle = a
            last_change_tick = now
            vel[i] = v
            continue

        dt_s = elapsed_ms * 0.001
        raw = d_angle / dt_s
        alpha = _lpf_alpha(lpf_hz, dt_s)
        v += alpha * (raw - v)
        last_angle = a
        last_change_tick = now
        vel[i] = v

    return vel


# --------------------------------------------------------------------------- #
# Onset + side auto-detection (there are no markers in these logs)            #
# --------------------------------------------------------------------------- #
def detect_onset_and_side(trial: TrialData,
                          rh_flex: np.ndarray,
                          lh_flex: np.ndarray,
                          event_code: int,
                          onset_thresh_dps: float = 120.0,
                          settle_s: float = 0.3):
    """Heuristic: the perturbation is the largest hip-velocity excursion in the
    harmful direction. Returns (onset_index, side_code). harmful flex direction
    is + for ES slip / trip, - for LS slip."""
    harmful = -1.0 if event_code == PERT_LS_SLIP else +1.0
    t = trial.time_s
    valid = t >= settle_s
    rh = harmful * rh_flex
    lh = harmful * lh_flex
    rh_masked = np.where(valid, rh, -np.inf)
    lh_masked = np.where(valid, lh, -np.inf)

    rh_peak = np.max(rh_masked)
    lh_peak = np.max(lh_masked)
    side = LEG_RIGHT if rh_peak >= lh_peak else LEG_LEFT
    sig = rh if side == LEG_RIGHT else lh
    peak_idx = int(np.argmax(np.where(valid, sig, -np.inf)))

    # walk back to the first upward crossing of the onset threshold.
    onset_idx = peak_idx
    for i in range(peak_idx, -1, -1):
        if sig[i] < onset_thresh_dps:
            onset_idx = i + 1
            break
        onset_idx = i
    onset_idx = min(onset_idx, len(t) - 1)
    return onset_idx, side


# --------------------------------------------------------------------------- #
# Ground-truth perturbation labels (*_events.csv)                             #
# --------------------------------------------------------------------------- #
def events_path_for(robot_csv_path: str) -> str | None:
    """Sibling *_events.csv for a *_robot.csv path, if it exists."""
    p = Path(robot_csv_path)
    suffix = "_robot.csv"
    if not p.name.endswith(suffix):
        return None
    cand = p.with_name(p.name[: -len(suffix)] + "_events.csv")
    return str(cand) if cand.exists() else None


def load_labelled_events(events_csv_path: str) -> list[dict]:
    """Ground-truth perturbation onsets from an *_events.csv.

    Schema v2 logs carry `perturbation_onset` rows with
    perturbation_event_id / perturbation_type / affected_leg /
    latest_mcu_time_ms (typically 4 per trial). Older logs only have a bare
    `perturbation` label with no type/side, in which case those fields come
    back None and the caller must infer them. `perturbation_invalid` (operator
    rejected) and `perturbation_recovery` (marks recovery, not onset) rows are
    skipped.
    """
    df = pd.read_csv(events_csv_path)
    out: list[dict] = []
    for _, row in df.iterrows():
        if str(row.get("label", "")) not in ("perturbation_onset", "perturbation"):
            continue
        t_ms = row.get("latest_mcu_time_ms")
        if pd.isna(t_ms):
            continue
        type_str = str(row.get("perturbation_type", "")).strip().lower()
        side_str = str(row.get("affected_leg", "")).strip().lower()
        out.append({
            "event_id": row.get("perturbation_event_id") or f"row{int(row.name)}",
            "mcu_time_ms": float(t_ms),
            "event_code": EVENT_TYPE_STR_TO_CODE.get(type_str),
            "side": SIDE_STR_TO_CODE.get(side_str),
        })
    out.sort(key=lambda e: e["mcu_time_ms"])
    return out


def resolve_events(trial: TrialData, raw_events: list[dict]) -> list[dict]:
    """Map labelled events onto trial sample indices, dropping any whose
    type/side could not be resolved (prints a warning so it's not silent)."""
    resolved = []
    for e in raw_events:
        if e["event_code"] is None or e["side"] is None:
            print(f"WARN: {e['event_id']}: unrecognized type/side "
                  f"(type={e.get('event_code')}, side={e.get('side')}) - skipped")
            continue
        tick = e["mcu_time_ms"] - trial.time_ms_origin
        onset_idx = int(np.argmin(np.abs(trial.time_ms - tick)))
        resolved.append({
            "event_id": e["event_id"],
            "event_code": e["event_code"],
            "side": e["side"],
            "onset_idx": onset_idx,
        })
    resolved.sort(key=lambda e: e["onset_idx"])
    return resolved


# --------------------------------------------------------------------------- #
# Kinematics-based onset re-anchoring (real-detector simulation)              #
# --------------------------------------------------------------------------- #
# NOTE ON AN EARLIER, DISCARDED VERSION OF THIS IDEA:
# The first attempt at this also re-decided event TYPE from kinematics --
# "whichever leg/instant has the single largest |qdot| in a +-1-stride window
# near the label". That failed badly: since ordinary stride swing peaks
# (~130-320 deg/s) are the same order of magnitude as a labelled slip's own
# excursion (see the qdot-vs-baseline finding earlier in this analysis), a
# ~1s-wide window almost always contains a bigger *unrelated* normal swing
# peak right next to the real, sometimes-smaller slip excursion -- so most
# LS-slip labels got silently relabelled ES-slip. Re-deciding type from raw
# magnitude is not trustworthy at this window width.
#
# This version instead KEEPS the label's perturbation_type/affected_leg
# (the experimenter knows which physical mechanism they triggered and on
# which leg -- a real detector fed by that same trigger signal would know it
# too) and uses local kinematics ONLY to correct the ONSET INSTANT: search
# near the label for where that leg's velocity actually crosses into the
# sign the type's primary_enable condition requires, instead of trusting the
# human keypress tick. This directly targets the mechanism proven earlier
# (sign-correct-at-onset <-> torque-delivered was 37/37 vs 35/39).
def snap_slip_onset(trial: TrialData, event_code: int, side: int, label_time_s: float,
                    qdot: np.ndarray | None = None,
                    onset_thresh_dps: float = 80.0,
                    search_back_s: float = 0.7,
                    search_fwd_s: float = 0.3,
                    vel_lpf_hz: float = 15.0) -> int | None:
    """For a KNOWN ES/LS-slip event_code+side, find the true onset near the
    label: the required-sign extremum in the search window, walked back to
    its threshold-crossing rising edge. Returns None if the required sign
    never clears onset_thresh_dps anywhere in the window."""
    t = trial.time_s
    if qdot is None:
        angle = trial.rh_angle if side == LEG_RIGHT else trial.lh_angle
        qdot = firmware_velocity(trial.time_ms, angle, vel_lpf_hz)

    lo = max(0, int(np.searchsorted(t, label_time_s - search_back_s)))
    hi = min(len(t), int(np.searchsorted(t, label_time_s + search_fwd_s)) + 1)
    if hi - lo < 2:
        return None

    required_sign = +1.0 if event_code == PERT_ES_SLIP else -1.0
    win = np.arange(lo, hi)
    signed = required_sign * qdot[win]
    i_peak = int(win[np.argmax(signed)])
    if required_sign * qdot[i_peak] < onset_thresh_dps:
        return None

    onset_idx = i_peak
    for i in range(i_peak, lo - 1, -1):
        if required_sign * qdot[i] < onset_thresh_dps:
            onset_idx = i + 1
            break
        onset_idx = i
    return max(onset_idx, lo)


def resolve_events_snap_onset(trial: TrialData, raw_events: list[dict],
                              **kwargs) -> list[dict]:
    """resolve_events()-compatible output: same type/side as the label, but
    ES/LS onset is re-anchored to the real velocity crossing via
    snap_slip_onset() (falls back to the label instant if no qualifying
    crossing is found nearby). TRIP keeps the label instant unmodified --
    its ACTIVE point is already re-anchored separately, to foot contact."""
    resolved = []
    rh_qdot = firmware_velocity(trial.time_ms, trial.rh_angle, kwargs.get("vel_lpf_hz", 15.0))
    lh_qdot = firmware_velocity(trial.time_ms, trial.lh_angle, kwargs.get("vel_lpf_hz", 15.0))
    for e in raw_events:
        if e["event_code"] is None or e["side"] is None:
            print(f"WARN: {e['event_id']}: unrecognized type/side - skipped")
            continue
        tick = e["mcu_time_ms"] - trial.time_ms_origin
        label_t = tick / 1000.0
        label_onset_idx = int(np.argmin(np.abs(trial.time_ms - tick)))

        onset_idx = label_onset_idx
        if e["event_code"] in (PERT_ES_SLIP, PERT_LS_SLIP):
            qdot = rh_qdot if e["side"] == LEG_RIGHT else lh_qdot
            snapped = snap_slip_onset(trial, e["event_code"], e["side"], label_t,
                                      qdot=qdot, **kwargs)
            if snapped is not None:
                onset_idx = snapped
            else:
                print(f"WARN: {e['event_id']}: no qualifying velocity crossing near label "
                     f"- kept label instant")

        resolved.append({"event_id": e["event_id"], "event_code": e["event_code"],
                         "side": e["side"], "onset_idx": onset_idx})
    resolved.sort(key=lambda e: e["onset_idx"])
    return resolved


# --------------------------------------------------------------------------- #
# Reaction controller port (state machine from section 11 of the .c)          #
# --------------------------------------------------------------------------- #
@dataclass
class ReactionWindow:
    """One triggered (or refractory-skipped) reaction, matching one labelled
    perturbation_onset row."""
    event_id: str
    event_code: int
    side: int
    onset_idx: int
    active_idx: int
    stop_idx: int
    com_scale: float
    fired: bool  # False = DampingAssist_Trigger would have ignored it
                 # (state != IDLE, i.e. still inside the previous
                 # ACTIVE+REFRACTORY window)


@dataclass
class SimResult:
    time_s: np.ndarray
    rh_qdot: np.ndarray
    lh_qdot: np.ndarray
    rh_flex: np.ndarray
    lh_flex: np.ndarray
    rh_tau_raw: np.ndarray
    lh_tau_raw: np.ndarray
    rh_tau_cmd: np.ndarray
    lh_tau_cmd: np.ndarray
    rh_B: np.ndarray
    lh_B: np.ndarray
    angle_scale: np.ndarray
    windows: list[ReactionWindow]
    # back-compat convenience: the first FIRED window (or first window if none
    # fired), same fields as before multi-event support existed.
    onset_idx: int
    active_idx: int
    stop_idx: int
    side: int
    event_code: int
    com_scale: float


def _resolve_active_idx(trial: TrialData, params: FirmwareParams, event_code: int,
                        primary: int, onset_idx: int,
                        trip_use_foot_contact: bool) -> int:
    t = trial.time_s
    n = len(t)
    if event_code != PERT_TRIP:
        return onset_idx
    perturbed_foot = trial.rh_foot if primary == LEG_RIGHT else trial.lh_foot
    active_idx = None
    if trip_use_foot_contact:
        for i in range(onset_idx + 1, n):
            if perturbed_foot[i] and not perturbed_foot[i - 1]:
                active_idx = i
                break
    if active_idx is None:  # fallback delay
        t_active = t[onset_idx] + params.trip_fallback_delay_ms / 1000.0
        cand = np.where(t >= t_active)[0]
        active_idx = int(cand[0]) if len(cand) else n - 1
    return active_idx


def _run_reaction(trial: TrialData, params: FirmwareParams, event_code: int, side: int,
                  onset_idx: int, rh_qdot: np.ndarray, lh_qdot: np.ndarray,
                  rh_flex: np.ndarray, lh_flex: np.ndarray, angle_scale: np.ndarray,
                  rh_tau: np.ndarray, lh_tau: np.ndarray, rh_B: np.ndarray, lh_B: np.ndarray,
                  trip_use_foot_contact: bool) -> tuple[int, int, float]:
    """Port of _StartReaction()/_RunReactionController() for ONE trigger.
    Writes tau/B into the [active_idx, stop_idx) window of the shared arrays
    (which are otherwise left untouched) and returns (active_idx, stop_idx,
    com_scale).

    Stop rule: pure sign-change (discrete Intermediate Value Theorem) on
    consecutive primary-leg qdot samples -- matches
    damping_assist_control.c's manual-detector _PrimaryVelocityReversed()
    configuration. No zero_cross_threshold_dps margin, and
    params.max_active_ms is NOT used as a fallback cap (see the CAUTION
    comment at that function in the .c): without it, ACTIVE can run until
    the next natural gait-cycle zero-crossing rather than a bounded time."""
    t = trial.time_s
    time_ms = trial.time_ms
    n = len(t)

    primary = side
    secondary = LEG_LEFT if primary == LEG_RIGHT else LEG_RIGHT

    com_scale = 1.0
    if event_code == PERT_TRIP and params.trip_com_scaling_enable:
        com_scale = 1.0 + params.trip_com_gain_per_mps * abs(params.manual_mean_com_velocity_mps)

    active_idx = _resolve_active_idx(trial, params, event_code, primary, onset_idx,
                                     trip_use_foot_contact)
    active_tick = int(time_ms[active_idx])
    stop_idx = n
    prev_p_flex = None  # no sample yet -> first post-guard tick only records a baseline

    for i in range(active_idx, n):
        elapsed = int(time_ms[i]) - active_tick

        p_flex = rh_flex[i] if primary == LEG_RIGHT else lh_flex[i]
        s_flex = rh_flex[i] if secondary == LEG_RIGHT else lh_flex[i]
        p_qdot = rh_qdot[i] if primary == LEG_RIGHT else lh_qdot[i]
        s_qdot = rh_qdot[i] if secondary == LEG_RIGHT else lh_qdot[i]

        if event_code == PERT_ES_SLIP:
            B_p, B_s = params.k_es_impact, params.k_es_stance * angle_scale[i]
            p_en, s_en = (p_flex > 0.0), (s_flex < 0.0)
        elif event_code == PERT_LS_SLIP:
            B_p, B_s = params.k_ls_slip, params.k_ls_stance * angle_scale[i]
            p_en, s_en = (p_flex < 0.0), (s_flex > 0.0)
        else:  # trip
            B_p, B_s = params.k_trip_impact * com_scale, params.k_trip_stance
            p_en, s_en = (p_flex > 0.0), (s_flex < 0.0)

        p_sign = params.right_torque_sign if primary == LEG_RIGHT else params.left_torque_sign
        s_sign = params.right_torque_sign if secondary == LEG_RIGHT else params.left_torque_sign

        tau_p = p_sign * (-B_p * p_qdot * DEG2RAD) if p_en else 0.0
        tau_s = s_sign * (-B_s * s_qdot * DEG2RAD) if s_en else 0.0

        if primary == LEG_RIGHT:
            rh_tau[i], lh_tau[i] = tau_p, tau_s
            rh_B[i], lh_B[i] = (B_p if p_en else 0.0), (B_s if s_en else 0.0)
        else:
            lh_tau[i], rh_tau[i] = tau_p, tau_s
            lh_B[i], rh_B[i] = (B_p if p_en else 0.0), (B_s if s_en else 0.0)

        # Stop: pure sign change on consecutive primary-leg qdot samples,
        # once elapsed >= min_active_before_stop_ms (mirrors the .c's
        # short-circuit: prev_p_flex only starts being tracked once the
        # guard opens, so the first post-guard tick can't itself report a
        # reversal -- it just records the baseline to compare against).
        if elapsed >= params.min_active_before_stop_ms:
            if prev_p_flex is not None and (prev_p_flex * p_flex) < 0.0:
                stop_idx = i + 1
                break
            prev_p_flex = p_flex

    return active_idx, min(stop_idx, n - 1), com_scale


def simulate_firmware(trial: TrialData, params: FirmwareParams, event_code: int,
                      side: int | None = None, onset_idx: int | None = None,
                      trip_use_foot_contact: bool = True,
                      events: list[dict] | None = None) -> SimResult:
    """Run the reaction-controller port over a trial.

    Legacy single-event mode (events=None): one perturbation, onset/side
    auto-detected from the largest hip-velocity spike if not given (used by
    --no-events and by the live tuning GUI).

    Multi-event mode (events=[{event_id, event_code, side, onset_idx}, ...],
    time-ordered): every labelled onset from *_events.csv is simulated in
    sequence with the same IDLE-gating DampingAssist_Trigger() applies in the
    .c — an onset that arrives before the previous reaction's
    ACTIVE + REFRACTORY window has ended is recorded but marked fired=False
    and produces no torque.
    """
    t = trial.time_s
    time_ms = trial.time_ms
    n = len(t)

    rh_qdot = firmware_velocity(time_ms, trial.rh_angle, params.vel_lpf_hz)
    lh_qdot = firmware_velocity(time_ms, trial.lh_angle, params.vel_lpf_hz)

    rh_flex = params.right_flexion_sign * rh_qdot
    lh_flex = params.left_flexion_sign * lh_qdot

    angle_scale = 1.0 + params.angle_gain_per_deg * np.abs(trial.rh_angle - trial.lh_angle)

    rh_tau = np.zeros(n)
    lh_tau = np.zeros(n)
    rh_B = np.zeros(n)
    lh_B = np.zeros(n)

    if events is None:
        if onset_idx is None or side is None:
            auto_onset, auto_side = detect_onset_and_side(trial, rh_flex, lh_flex, event_code)
            onset_idx = auto_onset if onset_idx is None else onset_idx
            side = auto_side if side is None else side
        events = [{"event_id": "auto", "event_code": event_code, "side": side,
                   "onset_idx": onset_idx}]

    windows: list[ReactionWindow] = []
    busy_until_tick = -1  # ticks are >= 0; nothing is busy at trial start
    for ev in events:
        onset_i = ev["onset_idx"]
        fired = int(time_ms[onset_i]) >= busy_until_tick
        if fired:
            active_i, stop_i, com_scale = _run_reaction(
                trial, params, ev["event_code"], ev["side"], onset_i,
                rh_qdot, lh_qdot, rh_flex, lh_flex, angle_scale,
                rh_tau, lh_tau, rh_B, lh_B, trip_use_foot_contact,
            )
            busy_until_tick = int(time_ms[stop_i]) + params.refractory_ms
        else:
            active_i, stop_i, com_scale = onset_i, onset_i, 1.0
        windows.append(ReactionWindow(
            event_id=str(ev["event_id"]), event_code=ev["event_code"], side=ev["side"],
            onset_idx=onset_i, active_idx=active_i, stop_idx=stop_i,
            com_scale=com_scale, fired=fired,
        ))

    rh_tau_raw = rh_tau.copy()
    lh_tau_raw = lh_tau.copy()
    if params.enable_torque_clamp:
        lim = abs(params.torque_limit_nm)
        rh_tau = np.clip(rh_tau, -lim, lim)
        lh_tau = np.clip(lh_tau, -lim, lim)

    fired_windows = [w for w in windows if w.fired] or windows
    first = fired_windows[0]

    return SimResult(
        time_s=t, rh_qdot=rh_qdot, lh_qdot=lh_qdot, rh_flex=rh_flex, lh_flex=lh_flex,
        rh_tau_raw=rh_tau_raw, lh_tau_raw=lh_tau_raw, rh_tau_cmd=rh_tau, lh_tau_cmd=lh_tau,
        rh_B=rh_B, lh_B=lh_B, angle_scale=angle_scale, windows=windows,
        onset_idx=first.onset_idx, active_idx=first.active_idx, stop_idx=first.stop_idx,
        side=first.side, event_code=first.event_code, com_scale=first.com_scale,
    )


# --------------------------------------------------------------------------- #
# Plotting                                                                    #
# --------------------------------------------------------------------------- #
def _k_primary_secondary(params: FirmwareParams, event_code: int) -> tuple[float, float]:
    """The two UNSCALED coefficients _RunReactionController() picks for a
    type: (k_primary=impact/slipping leg, k_secondary=stance leg -- the
    stance one still gets multiplied by angle_scale before use, the primary
    one never does)."""
    if event_code == PERT_ES_SLIP:
        return params.k_es_impact, params.k_es_stance
    if event_code == PERT_LS_SLIP:
        return params.k_ls_slip, params.k_ls_stance
    return params.k_trip_impact, params.k_trip_stance


def plot_result(trial: TrialData, res: SimResult, params: FirmwareParams,
                title_extra: str = "", xlim: tuple[float, float] | None = None,
                onset_label: str = "onset (events.csv)"):
    """xlim: if given (t0, t1) in seconds, all 4 panels are cropped to that
    window instead of showing the whole trial (and the small torque inset,
    which would be redundant, is skipped).
    onset_label: legend text for the black dashed onset marker -- say where it
    actually came from (raw label tick vs --auto-classify's velocity-sign-
    crossing snap); the marker position itself always reflects whatever
    onset_idx simulate_firmware was given, this only fixes the caption."""
    from matplotlib.figure import Figure

    fig = Figure(figsize=(11, 9.5), dpi=100)
    ax_ang = fig.add_subplot(411)
    ax_vel = fig.add_subplot(412, sharex=ax_ang)
    ax_tau = fig.add_subplot(413, sharex=ax_ang)
    ax_b = fig.add_subplot(414, sharex=ax_ang)

    t = res.time_s
    t_active = t[res.active_idx]
    t_stop = t[res.stop_idx]

    def marks(ax):
        onset_labelled = active_labelled = stop_labelled = skipped_labelled = False
        for w in res.windows:
            t_on, t_ac, t_st = t[w.onset_idx], t[w.active_idx], t[w.stop_idx]
            if not w.fired:
                ax.axvline(t_on, ls="--", color="0.6", lw=1.4, alpha=0.6,
                          label=None if skipped_labelled else "skipped (refractory)")
                skipped_labelled = True
                continue
            ax.axvline(t_on, ls="--", color="k", lw=1,
                      label=None if onset_labelled else onset_label)
            onset_labelled = True
            if w.event_code == PERT_TRIP:
                ax.axvline(t_ac, ls=":", color="tab:purple", lw=1,
                          label=None if active_labelled else "ACTIVE (foot contact)")
                active_labelled = True
            ax.axvline(t_st, ls="-.", color="tab:red", lw=1,
                      label=None if stop_labelled else "stop (vel reversal)")
            stop_labelled = True
            # stop_idx is the firmware's first ALREADY-IDLE tick (tau/B_eff
            # are 0 there by construction -- the reaction is still valid on
            # the tick where reversal is detected, per _RunReactionController
            # in the .c). Shading through t_st would visually include that
            # zero point next to still-elevated angle/qdot traces, which
            # reads as a mismatch that isn't really there -- shade only the
            # genuinely active samples instead.
            t_last_active = t[max(w.active_idx, w.stop_idx - 1)]
            ax.axvspan(t_ac, t_last_active, color="tab:orange", alpha=0.10)

    ax_ang.plot(t, trial.rh_angle, label="RH angle", color="tab:blue")
    ax_ang.plot(t, trial.lh_angle, label="LH angle", color="tab:green")
    ax_ang.plot(t, np.abs(trial.rh_angle - trial.lh_angle), ls=":", lw=1.3,
               color="tab:gray", label="|RH-LH| (drives angle_scale)")
    ax_ang.set_ylabel("hip angle [deg]")
    marks(ax_ang)
    ax_ang.legend(loc="upper right", fontsize=8)
    ax_ang.grid(True, alpha=0.2)

    ax_vel.plot(t, res.rh_qdot, label="RH qdot (firmware)", color="tab:blue")
    ax_vel.plot(t, res.lh_qdot, label="LH qdot (firmware)", color="tab:green")
    if trial.rh_vel_logged is not None:
        ax_vel.plot(t, trial.rh_vel_logged, color="tab:blue", alpha=0.3, lw=0.8,
                    label="RH qdot (logged 20Hz)")
    ax_vel.axhline(0, color="0.6", lw=0.6)
    ax_vel.set_ylabel("hip qdot [deg/s]")
    marks(ax_vel)
    ax_vel.legend(loc="upper right", fontsize=8)
    ax_vel.grid(True, alpha=0.2)

    ax_tau.plot(t, res.rh_tau_raw, label="RH expected torque", color="tab:blue")
    ax_tau.plot(t, res.lh_tau_raw, label="LH expected torque", color="tab:green")
    if params.enable_torque_clamp:
        ax_tau.plot(t, res.rh_tau_cmd, ls="--", color="tab:blue", alpha=0.6, label="RH clamped")
        ax_tau.plot(t, res.lh_tau_cmd, ls="--", color="tab:green", alpha=0.6, label="LH clamped")
    ax_tau.axhline(0, color="0.6", lw=0.6)
    ax_tau.set_ylabel("torque [Nm]")
    marks(ax_tau)
    ax_tau.legend(loc="upper right", fontsize=8)
    ax_tau.grid(True, alpha=0.2)

    # Zoom inset on the torque panel: the event is short vs. the whole trial,
    # so show the expected torque during the ACTIVE window up close. Skipped
    # when the caller already cropped the whole figure via xlim (redundant).
    pad = 0.35
    x0, x1 = t_active - pad, t_stop + pad
    win = (t >= x0) & (t <= x1)
    if xlim is None and np.any(win):
        axz = ax_tau.inset_axes([0.60, 0.50, 0.38, 0.46])
        axz.plot(t, res.rh_tau_raw, color="tab:blue")
        axz.plot(t, res.lh_tau_raw, color="tab:green")
        axz.axhline(0, color="0.6", lw=0.6)
        t_last_active_0 = t[max(res.active_idx, res.stop_idx - 1)]
        axz.axvspan(t_active, t_last_active_0, color="tab:orange", alpha=0.12)
        axz.axvline(t_active, ls=":", color="tab:purple", lw=1)
        axz.axvline(t_stop, ls="-.", color="tab:red", lw=1)
        axz.set_xlim(x0, x1)
        span = float(np.max(np.abs(np.concatenate([res.rh_tau_raw[win], res.lh_tau_raw[win], [0.05]]))))
        axz.set_ylim(-1.15 * span, 1.15 * span)
        n_fired = sum(1 for w in res.windows if w.fired)
        axz.set_title(f"event zoom (1st of {n_fired} fired)" if n_fired > 1 else "event zoom",
                     fontsize=7)
        axz.tick_params(labelsize=6)
        ax_tau.indicate_inset_zoom(axz, edgecolor="0.4")

    ax_b.plot(t, res.rh_B, label="RH B_eff", color="tab:blue")
    ax_b.plot(t, res.lh_B, label="LH B_eff", color="tab:green")
    ax_b.plot(t, res.angle_scale, label="inter-leg angle scale (multiplier)", color="tab:gray", lw=0.8)

    # Unscaled k_primary / k_secondary reference, flat over each fired
    # window: primary's actual B_eff should sit exactly on its dashed line
    # (no scaling applied); secondary's should sit ABOVE its dashed line by
    # exactly the angle_scale multiplier at that instant.
    kp_labelled = ks_labelled = False
    for w in res.windows:
        if not w.fired:
            continue
        k_p, k_s = _k_primary_secondary(params, w.event_code)
        t_ac, t_st = t[w.active_idx], t[w.stop_idx]
        primary_color = "tab:blue" if w.side == LEG_RIGHT else "tab:green"
        secondary_color = "tab:green" if w.side == LEG_RIGHT else "tab:blue"
        ax_b.hlines(k_p, t_ac, t_st, color=primary_color, ls=":", lw=1.6, alpha=0.9,
                   label=None if kp_labelled else "k_primary (unscaled)")
        kp_labelled = True
        ax_b.hlines(k_s, t_ac, t_st, color=secondary_color, ls=":", lw=1.6, alpha=0.9,
                   label=None if ks_labelled else "k_secondary (unscaled)")
        ks_labelled = True

    ax_b.set_ylabel("B_eff / scale [Nm*s/rad]")
    ax_b.set_xlabel("time [s]")
    marks(ax_b)
    ax_b.legend(loc="upper right", fontsize=7.5)
    ax_b.grid(True, alpha=0.2)

    if xlim is not None:
        for ax in (ax_ang, ax_vel, ax_tau, ax_b):
            ax.set_xlim(*xlim)
        win_mask = (t >= xlim[0]) & (t <= xlim[1])
        rh_peak = float(np.max(np.abs(res.rh_tau_raw[win_mask]))) if np.any(win_mask) else 0.0
        lh_peak = float(np.max(np.abs(res.lh_tau_raw[win_mask]))) if np.any(win_mask) else 0.0
        shown_windows = [w for w in res.windows
                         if t[w.onset_idx] >= xlim[0] and t[w.onset_idx] <= xlim[1]]
    else:
        rh_peak = float(np.max(np.abs(res.rh_tau_raw)))
        lh_peak = float(np.max(np.abs(res.lh_tau_raw)))
        shown_windows = res.windows

    n_fired = sum(1 for w in res.windows if w.fired)
    n_total = len(res.windows)

    def _summarize(w: ReactionWindow) -> str:
        side_ch = "R" if w.side == LEG_RIGHT else "L"
        name = EVENT_CODE_TO_NAME[w.event_code].split()[0]
        if not w.fired:
            return f"{w.event_id}:{name}/{side_ch} skipped"
        dur_ms = (t[w.stop_idx] - t[w.active_idx]) * 1000.0
        return f"{w.event_id}:{name}/{side_ch} {dur_ms:.0f}ms"

    header = (f"zoom {xlim[0]:.2f}-{xlim[1]:.2f}s | in-view peak |tau| RH={rh_peak:.2f} LH={lh_peak:.2f} Nm"
             if xlim is not None else
             f"{n_total} labelled perturbation(s), {n_fired} fired | peak |tau| RH={rh_peak:.2f} LH={lh_peak:.2f} Nm")
    title = (
        f"{Path(trial.path).name}\n{header}\n"
        + " | ".join(_summarize(w) for w in shown_windows)
    )
    if title_extra:
        title += f" | {title_extra}"
    fig.suptitle(title, fontsize=8)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    return fig


# --------------------------------------------------------------------------- #
# CLI                                                                         #
# --------------------------------------------------------------------------- #
def _event_from_name_or_path(event_arg: str | None, path: str) -> int:
    if event_arg:
        key = event_arg.upper().replace("-", "_")
        if key in ("ES_SLIP", "ES"):
            return PERT_ES_SLIP
        if key in ("LS_SLIP", "LS"):
            return PERT_LS_SLIP
        if key in ("TRIP", "EARLY_SWING_TRIP"):
            return PERT_TRIP
    name = Path(path).name.lower()
    if "trip" in name:
        return PERT_TRIP
    return PERT_ES_SLIP  # slip default; cannot tell ES vs LS from the file name


def _side_from_arg(side_arg: str | None):
    if not side_arg:
        return None
    s = side_arg.upper()
    if s in ("R", "RIGHT"):
        return LEG_RIGHT
    if s in ("L", "LEFT"):
        return LEG_LEFT
    return None


def _params_from_defaults() -> FirmwareParams:
    if _HAVE_GUI_PARAMS:
        return FirmwareParams.from_gui_params(_GuiParams())
    return FirmwareParams()


def _events_for_trial(path: str, trial: TrialData, args) -> list[dict] | None:
    """events.csv onsets for this trial, unless the user opted out
    (--no-events) or gave an explicit manual --onset override.

    Default: trust the label's type/side/instant (resolve_events).
    --auto-classify: keep the label's type/side (the experimenter's own
    trigger mechanism), but re-anchor ES/LS onset to the real velocity
    crossing near the label (resolve_events_snap_onset) instead of the
    human keypress tick -- a stand-in for a real online detector that
    watches the same kinematics but doesn't wait on a human reaction time."""
    if getattr(args, "no_events", False):
        return None
    if getattr(args, "onset", None) is not None:
        return None
    ev_path = events_path_for(path)
    if not ev_path:
        return None
    raw = load_labelled_events(ev_path)
    if getattr(args, "auto_classify", False):
        resolved = resolve_events_snap_onset(trial, raw)
    else:
        resolved = resolve_events(trial, raw)
    return resolved or None


def _estimate_cycle_s(trial: TrialData, t_center: float, side: int,
                      search_pad_s: float = 4.0, default_s: float = 1.1) -> float:
    """Rough stride-period estimate from local hip-angle peak spacing, used
    only to size a 'show N gait cycles' zoom window (not for control)."""
    t = trial.time_s
    angle = trial.rh_angle if side == LEG_RIGHT else trial.lh_angle
    mask = (t >= t_center - search_pad_s) & (t <= t_center + search_pad_s)
    idx = np.where(mask)[0]
    if len(idx) < 20:
        return default_s
    a, tt = angle[idx], t[idx]
    peaks = [i for i in range(1, len(a) - 1) if a[i] > a[i - 1] and a[i] >= a[i + 1]]
    if len(peaks) < 2:
        return default_s
    periods = np.diff(tt[peaks])
    periods = periods[periods > 0.4]  # drop spurious sub-stride wiggles
    return float(np.median(periods)) if len(periods) else default_s


def zoom_xlim_for_window(trial: TrialData, w: ReactionWindow, cycles: float) -> tuple[float, float]:
    """~`cycles` gait cycles centered on the event's ACTIVE start (where the
    damping actually happens), clipped to the trial. For ES/LS this equals
    the onset (active_idx == onset_idx), so it's unchanged for those; for
    trip it's the foot-contact point, which can trail onset by several
    hundred ms -- centering on onset there would often crop ACTIVE itself
    off the edge of the window."""
    t = trial.time_s
    t_center = t[w.active_idx]
    cycle_s = _estimate_cycle_s(trial, t_center, w.side)
    half = cycles * cycle_s / 2.0
    return max(0.0, t_center - half), min(float(t[-1]), t_center + half)


def _apply_param_overrides(params: FirmwareParams, args) -> None:
    """CLI-level overrides applied on top of _params_from_defaults().
    Applied in order so more specific flags win: --b-all, then
    --b-primary/--b-secondary (each overwrites only its own role across all
    3 event types), so e.g. `--b-all 0.3 --b-primary 50` gives every stance
    coefficient 0.3 and every impact/slipping coefficient 50."""
    b_all = getattr(args, "b_all", None)
    if b_all is not None:
        params.k_es_impact = params.k_es_stance = float(b_all)
        params.k_ls_slip = params.k_ls_stance = float(b_all)
        params.k_trip_impact = params.k_trip_stance = float(b_all)

    b_primary = getattr(args, "b_primary", None)
    if b_primary is not None:
        params.k_es_impact = params.k_ls_slip = params.k_trip_impact = float(b_primary)

    b_secondary = getattr(args, "b_secondary", None)
    if b_secondary is not None:
        params.k_es_stance = params.k_ls_stance = params.k_trip_stance = float(b_secondary)


def run_single(path: str, args) -> "object":
    trial = load_robot_csv(path)
    params = _params_from_defaults()
    _apply_param_overrides(params, args)
    if args.trip_delay:
        params.trip_fallback_delay_ms = int(args.trip_delay)

    events = _events_for_trial(path, trial, args)
    if events:
        res = simulate_firmware(trial, params, PERT_NONE, events=events,
                                trip_use_foot_contact=not args.trip_delay)
    else:
        event_code = _event_from_name_or_path(args.event, path)
        side = _side_from_arg(args.side)
        onset_idx = None
        if args.onset is not None and str(args.onset).lower() != "auto":
            onset_idx = int(np.argmin(np.abs(trial.time_s - float(args.onset))))
        res = simulate_firmware(trial, params, event_code, side=side, onset_idx=onset_idx,
                                trip_use_foot_contact=not args.trip_delay)

    onset_label = ("onset (auto-classify snap)" if events and getattr(args, "auto_classify", False)
                  else "onset (events.csv)" if events
                  else "onset (heuristic auto-detect)")

    zoom_cycles = getattr(args, "zoom_cycles", None)
    if zoom_cycles:
        targets = [w for w in res.windows if w.fired]
        want_id = getattr(args, "zoom_event", None)
        if want_id:
            targets = [w for w in targets if w.event_id == want_id]
        if not targets:
            print("no fired events to zoom on"
                 + (f" (event_id={want_id!r} not found)" if want_id else ""))
        out_dir = Path(args.save).parent if args.save else Path("torque_png")
        os.makedirs(out_dir, exist_ok=True)
        stem = Path(args.save).stem if args.save else Path(path).stem
        fig = None
        for w in targets:
            xlim = zoom_xlim_for_window(trial, w, zoom_cycles)
            fig = plot_result(trial, res, params, xlim=xlim, onset_label=onset_label,
                              title_extra=f"~{zoom_cycles:g} gait cycles around {w.event_id}")
            out = out_dir / f"{stem}_{w.event_id}_zoom.png"
            fig.savefig(out, dpi=110)
            print(f"saved {out}")
        return fig, res

    fig = plot_result(trial, res, params, onset_label=onset_label)
    if args.save:
        fig.savefig(args.save, dpi=110)
        print(f"saved {args.save}")
    return fig, res


def run_batch(folder: str, outdir: str, args):
    os.makedirs(outdir, exist_ok=True)
    csvs = sorted(Path(folder).glob("*_robot.csv"))
    if not csvs:
        print(f"no *_robot.csv in {folder}")
        return
    params = _params_from_defaults()
    _apply_param_overrides(params, args)
    if args.trip_delay:
        params.trip_fallback_delay_ms = int(args.trip_delay)
    summary = []
    for p in csvs:
        try:
            trial = load_robot_csv(str(p))
            events = _events_for_trial(str(p), trial, args)
            if events:
                res = simulate_firmware(trial, params, PERT_NONE, events=events,
                                        trip_use_foot_contact=not args.trip_delay)
                n_fired = sum(1 for w in res.windows if w.fired)
                event_label = f"{n_fired}/{len(events)} labelled fired"
                onset_label = ("onset (auto-classify snap)" if getattr(args, "auto_classify", False)
                              else "onset (events.csv)")
            else:
                event_code = _event_from_name_or_path(args.event, str(p))
                res = simulate_firmware(trial, params, event_code,
                                        trip_use_foot_contact=not args.trip_delay)
                event_label = EVENT_CODE_TO_NAME[event_code] + " (auto-detect, no events.csv)"
                onset_label = "onset (heuristic auto-detect)"
            fig = plot_result(trial, res, params, onset_label=onset_label)
            out = Path(outdir) / (p.stem + "_expected_torque.png")
            fig.savefig(out, dpi=110)
            rh_peak = float(np.max(np.abs(res.rh_tau_raw)))
            lh_peak = float(np.max(np.abs(res.lh_tau_raw)))
            summary.append((p.name, event_label, rh_peak, lh_peak))
            print(f"ok  {p.name}  {event_label}  peak RH={rh_peak:.2f} LH={lh_peak:.2f} Nm -> {out.name}")
        except Exception as exc:
            print(f"ERR {p.name}: {exc}")
    print(f"\n{len(summary)} trials -> {outdir}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="robot CSV path (single-trial mode)")
    ap.add_argument("--event", help="ES_SLIP | LS_SLIP | TRIP (default: infer from name)")
    ap.add_argument("--side", help="R | L primary/perturbed leg (default: auto-detect)")
    ap.add_argument("--onset", help="'auto' or onset time in seconds (default: auto)")
    ap.add_argument("--no-events", action="store_true",
                    help="ignore *_events.csv labels; use the old heuristic "
                         "(largest hip-velocity spike) instead")
    ap.add_argument("--auto-classify", action="store_true",
                    help="keep the events.csv label's type/side, but for ES/LS "
                         "slip re-anchor onset to the actual velocity-sign "
                         "crossing near the label instead of the human "
                         "keypress instant (simulates an online detector)")
    ap.add_argument("--trip-delay", type=float, default=None,
                    help="use fixed ms delay for trip ACTIVE instead of foot contact")
    ap.add_argument("--b-all", type=float, default=None,
                    help="override all 6 damping coefficients "
                         "(k_es_impact/stance, k_ls_slip/stance, k_trip_impact/stance) "
                         "to this single value [Nm*s/rad], in place of the 0.30 .c default")
    ap.add_argument("--b-primary", type=float, default=None,
                    help="override just the primary/impact/slipping-leg "
                         "coefficients (k_es_impact, k_ls_slip, k_trip_impact) "
                         "[Nm*s/rad]; applied after --b-all, so combine e.g. "
                         "--b-all 0.3 --b-primary 50 to tune impact leg only")
    ap.add_argument("--b-secondary", type=float, default=None,
                    help="override just the secondary/stance-leg coefficients "
                         "(k_es_stance, k_ls_stance, k_trip_stance) [Nm*s/rad]; "
                         "applied after --b-all, same combining rule as --b-primary")
    ap.add_argument("--zoom-cycles", type=float, default=None,
                    help="single-trial mode: instead of the whole-trial plot, save one "
                         "PNG per fired labelled event, x-cropped to ~N gait cycles "
                         "around its onset (e.g. --zoom-cycles 2)")
    ap.add_argument("--zoom-event",
                    help="with --zoom-cycles, only render this perturbation_event_id "
                         "(e.g. P002); default: every fired event in the trial")
    ap.add_argument("--save", help="save PNG to this path")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    ap.add_argument("--gui", action="store_true", help="drive the tuning GUI on this CSV")
    ap.add_argument("--batch", help="folder of *_robot.csv to render to PNGs")
    ap.add_argument("--outdir", default="torque_png", help="output dir for --batch")
    args = ap.parse_args(argv)

    if args.gui:
        if not args.csv:
            ap.error("--gui needs a CSV path")
        launch_gui(args.csv, args)
        return

    if not args.show:
        import matplotlib
        matplotlib.use("Agg")

    if args.batch:
        run_batch(args.batch, args.outdir, args)
        return

    if not args.csv:
        ap.error("provide a CSV path, or use --batch <folder>")

    fig, _ = run_single(args.csv, args)
    if args.show:
        import tkinter as tk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        root = tk.Tk()
        root.title("expected torque")
        canvas = FigureCanvasTkAgg(fig, master=root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        root.mainloop()


# --------------------------------------------------------------------------- #
# Optional: drive the existing tuning GUI on a real robot CSV                 #
# --------------------------------------------------------------------------- #
def launch_gui(csv_path: str, args):
    """Subclass the existing DampingTuningGUI so its sliders retune the
    firmware-faithful torque computed from a real ND_5 robot CSV."""
    import tkinter as tk
    from damping_tuning_gui import DampingTuningGUI

    trial = load_robot_csv(csv_path)
    event_code = _event_from_name_or_path(args.event, csv_path)
    forced_side = _side_from_arg(args.side)
    event_name = EVENT_CODE_TO_NAME[event_code]

    class RobotCsvGUI(DampingTuningGUI):
        _pipeline_ready = False

        def __init__(self, root):
            super().__init__(root)
            self._trial = trial
            self._forced_side = forced_side
            self.event_var.set(event_name)
            self.root.title(f"H10 damping — {Path(csv_path).name}")
            self._pipeline_ready = True
            self.update_plot()

        def update_plot(self):
            # Until fully constructed, defer to the base synthetic preview.
            if not getattr(self, "_pipeline_ready", False):
                return super().update_plot()

            gp = self.current_params()
            params = FirmwareParams.from_gui_params(gp)
            code = _event_from_name_or_path(self.event_var.get(), csv_path)
            side = self._forced_side
            if self.side_var.get() in ("Right", "Left"):
                side = LEG_RIGHT if self.side_var.get() == "Right" else LEG_LEFT
            res = simulate_firmware(self._trial, params, code, side=side)

            self.ax_vel.clear(); self.ax_tau.clear(); self.ax_scale.clear()
            t = res.time_s
            t_on, t_ac, t_st = t[res.onset_idx], t[res.active_idx], t[res.stop_idx]

            self.ax_vel.plot(t, res.rh_qdot, label="RH qdot")
            self.ax_vel.plot(t, res.lh_qdot, label="LH qdot")
            for x, ls in ((t_on, "--"), (t_ac, ":"), (t_st, "-.")):
                self.ax_vel.axvline(x, ls=ls, color="k", lw=0.8)
            self.ax_vel.axvspan(t_ac, t_st, color="tab:orange", alpha=0.1)
            self.ax_vel.set_ylabel("qdot [deg/s]"); self.ax_vel.grid(True, alpha=0.2)
            self.ax_vel.legend(loc="upper right", fontsize=8)

            self.ax_tau.plot(t, res.rh_tau_raw, label="RH torque")
            self.ax_tau.plot(t, res.lh_tau_raw, label="LH torque")
            if params.enable_torque_clamp:
                self.ax_tau.plot(t, res.rh_tau_cmd, ls="--", label="RH clamped")
                self.ax_tau.plot(t, res.lh_tau_cmd, ls="--", label="LH clamped")
            self.ax_tau.axhline(0, lw=0.8); self.ax_tau.set_ylabel("torque [Nm]")
            self.ax_tau.grid(True, alpha=0.2); self.ax_tau.legend(loc="upper right", fontsize=8)

            self.ax_scale.plot(t, res.rh_B, label="RH B_eff")
            self.ax_scale.plot(t, res.lh_B, label="LH B_eff")
            self.ax_scale.plot(t, res.angle_scale, label="angle scale")
            self.ax_scale.set_xlabel("time [s]"); self.ax_scale.set_ylabel("B_eff / scale")
            self.ax_scale.grid(True, alpha=0.2); self.ax_scale.legend(loc="upper right", fontsize=8)

            rh_peak = float(np.max(np.abs(res.rh_tau_raw)))
            lh_peak = float(np.max(np.abs(res.lh_tau_raw)))
            side_name = "RIGHT" if res.side == LEG_RIGHT else "LEFT"
            self.fig.suptitle(
                f"{EVENT_CODE_TO_NAME[code]} | primary {side_name} | "
                f"peak |tau| RH={rh_peak:.2f} LH={lh_peak:.2f} Nm (real CSV)"
            )
            self.fig.tight_layout(rect=[0, 0, 1, 0.96])
            self.canvas.draw_idle()
            self.status_var.set(
                f"Loaded {Path(csv_path).name} | firmware-faithful sim | "
                f"onset={t[res.onset_idx]:.2f}s side={side_name}"
            )

    root = tk.Tk()
    RobotCsvGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
