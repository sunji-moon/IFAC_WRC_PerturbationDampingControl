#!/usr/bin/env python3
"""Plot a recorded damping-controller trial (output of damping_live_logger.py).

Makes two figures from a *_damping.csv / *_events.csv pair:
  1. <stem>_event_example.png  - one triggered event, both legs:
     hip flexion velocity, damping coefficient B_eff, commanded torque vs.
     torque measured by H10 (estimated from motor current).
  2. <stem>_event_average.png  - every triggered event aligned at the trigger,
     mean +/- 1 std of perturbed-leg velocity and torque.

Usage:
    python plot_damping_trial.py <..._damping.csv> [--event N] [--outdir DIR]
The matching _events.csv is found next to the damping CSV.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

EVENT_NAMES = {1: "ES slip", 2: "LS slip", 3: "Early-swing trip"}
SIDE_NAMES = {1: "right", 2: "left"}
RIGHT, LEFT = "#1f77b4", "#2ca02c"


def load(damping_csv: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    d = pd.read_csv(damping_csv)
    d = d.drop_duplicates("mcu_time_ms").sort_values("mcu_time_ms")
    d["t"] = (d["mcu_time_ms"] - d["mcu_time_ms"].iloc[0]) / 1000.0
    events_csv = damping_csv.with_name(
        damping_csv.name.replace("_damping.csv", "_events.csv"))
    e = pd.read_csv(events_csv)
    t0 = d["mcu_time_ms"].iloc[0]
    trig = e[e["label"].str.endswith("_trigger")].copy()
    stop = e[e["label"] == "reaction_stop"].copy()
    trig["t"] = (trig["mcu_time_ms"] - t0) / 1000.0
    stop["t"] = (stop["mcu_time_ms"] - t0) / 1000.0
    trig = trig.reset_index(drop=True)
    trig["t_stop"] = [
        stop.loc[stop["t"] >= t, "t"].min() for t in trig["t"]]
    return d, trig


def plot_example(d: pd.DataFrame, trig: pd.DataFrame, k: int, out: Path,
                 title: str) -> None:
    ev = trig.iloc[k]
    t0, t1 = ev["t"] - 0.6, ev["t"] + 1.0
    w = d[(d["t"] >= t0) & (d["t"] <= t1)]
    side = int(ev["marker_event_side"])
    etype = EVENT_NAMES.get(int(ev["marker_event_type"]), "event")

    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for a in ax:
        a.axvline(ev["t"], color="k", ls="--", lw=1, label="trigger")
        if np.isfinite(ev["t_stop"]):
            a.axvspan(ev["t"], ev["t_stop"], color="orange", alpha=0.15,
                      label="controller active")
        a.grid(alpha=0.3)
    ax[0].plot(w["t"], w["right_flex_velocity_dps"], color=RIGHT, label="right hip")
    ax[0].plot(w["t"], w["left_flex_velocity_dps"], color=LEFT, label="left hip")
    ax[0].axhline(0, color="gray", lw=0.8)
    ax[0].set_ylabel("hip velocity [deg/s]\n(+ flexion)")
    ax[1].plot(w["t"], w["right_b_eff_nms_rad"], color=RIGHT, label="right")
    ax[1].plot(w["t"], w["left_b_eff_nms_rad"], color=LEFT, label="left")
    ax[1].set_ylabel("B_eff [N·m·s/rad]")
    ax[2].plot(w["t"], w["right_tau_cmd_nm"], color=RIGHT, label="right, commanded")
    ax[2].plot(w["t"], w["right_joint_torque_est_nm"], color=RIGHT, ls=":",
               label="right, measured")
    ax[2].plot(w["t"], w["left_tau_cmd_nm"], color=LEFT, label="left, commanded")
    ax[2].plot(w["t"], w["left_joint_torque_est_nm"], color=LEFT, ls=":",
               label="left, measured")
    ax[2].set_ylabel("hip torque [N·m]")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.legend(fontsize=8, loc="upper right")
    fig.suptitle(f"{title}\n{etype}, perturbed leg = {SIDE_NAMES.get(side, '?')}"
                 f" (event {k + 1} of {len(trig)})")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_average(d: pd.DataFrame, trig: pd.DataFrame, out: Path,
                 title: str) -> None:
    grid = np.arange(-0.3, 0.8, 0.01)
    vel, tau, meas, dur = [], [], [], []
    for _, ev in trig.iterrows():
        p = "right" if int(ev["marker_event_side"]) == 1 else "left"
        rel = d["t"].to_numpy() - ev["t"]
        if rel.min() > grid[0] or rel.max() < grid[-1]:
            continue
        vel.append(np.interp(grid, rel, d[f"{p}_flex_velocity_dps"]))
        tau.append(np.interp(grid, rel, d[f"{p}_tau_cmd_nm"]))
        meas.append(np.interp(grid, rel, d[f"{p}_joint_torque_est_nm"]))
        if np.isfinite(ev["t_stop"]):
            dur.append(ev["t_stop"] - ev["t"])
    vel, tau, meas = map(np.array, (vel, tau, meas))

    fig, ax = plt.subplots(2, 1, figsize=(8, 6.5), sharex=True)
    for a, series, lab, col in (
            (ax[0], [vel], ["flexion velocity"], ["#1f77b4"]),
            (ax[1], [tau, meas], ["commanded", "measured"], ["#d62728", "#7f7f7f"])):
        for s, l, c in zip(series, lab, col):
            m, sd = s.mean(0), s.std(0)
            a.plot(grid, m, color=c, label=l)
            a.fill_between(grid, m - sd, m + sd, color=c, alpha=0.2)
        a.axvline(0, color="k", ls="--", lw=1)
        a.axhline(0, color="gray", lw=0.8)
        a.grid(alpha=0.3)
        a.legend(fontsize=9)
    ax[0].set_ylabel("perturbed-leg hip\nvelocity [deg/s]")
    ax[1].set_ylabel("perturbed-leg hip\ntorque [N·m]")
    ax[1].set_xlabel("time from trigger [s]")
    txt = f"n = {len(vel)} events"
    if dur:
        txt += f", active duration {np.mean(dur) * 1000:.0f} ± {np.std(dur) * 1000:.0f} ms"
    fig.suptitle(f"{title}\nmean ± 1 std across events ({txt})")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("damping_csv", type=Path)
    ap.add_argument("--event", type=int, default=None,
                    help="1-based event to plot in the example figure "
                         "(default: the one with the largest commanded torque)")
    ap.add_argument("--outdir", type=Path, default=None)
    args = ap.parse_args()

    d, trig = load(args.damping_csv)
    if trig.empty:
        raise SystemExit("no trigger events found")
    outdir = args.outdir or args.damping_csv.parent
    outdir.mkdir(parents=True, exist_ok=True)
    stem = args.damping_csv.name.replace("_damping.csv", "")
    title = stem

    if args.event is None:
        peaks = []
        for _, ev in trig.iterrows():
            stop = ev["t_stop"] if np.isfinite(ev["t_stop"]) else ev["t"] + 0.5
            w = d[(d["t"] >= ev["t"]) & (d["t"] <= stop)]
            peaks.append(w[["left_tau_cmd_nm", "right_tau_cmd_nm"]].abs().to_numpy().max()
                         if len(w) else 0.0)
        k = int(np.argmax(peaks))
    else:
        k = args.event - 1

    plot_example(d, trig, k, outdir / f"{stem}_event_example.png", title)
    plot_average(d, trig, outdir / f"{stem}_event_average.png", title)
    print(f"wrote figures for {len(trig)} events to {outdir}")


if __name__ == "__main__":
    main()
