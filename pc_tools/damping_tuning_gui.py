"""
damping_tuning_gui.py

Interactive preview GUI for the XM/H10 continuous damping controller.

What it does
------------
1. Lets you tune ES-slip / LS-slip / early-swing-trip damping coefficients.
2. Recomputes the SAME main torque equations used in user_app.c:
       tau = -B_eff * qdot
3. Immediately plots approximate RH/LH torque waveforms.
4. Can use a synthetic perturbation waveform or load a CSV containing
   hip angle / hip angular-velocity data.
5. Shows the effective stance-leg angle scaling and trip COM scaling.
6. Can copy the current tuning parameters as C assignments.

What it does NOT do yet
-----------------------
- It does not write values directly into STM32CubeIDE Live Expressions.
- It does not implement COM estimation.
- It does not decode the XM USB custom-data protocol.

That hardware-live link can be added later once your USB/serial data path is fixed.

Run:
    python damping_tuning_gui.py

Dependencies:
    numpy, pandas, matplotlib
Tkinter is included with most standard Python installations.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from dataclasses import dataclass

import numpy as np
import pandas as pd

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


DEG2RAD = np.pi / 180.0


@dataclass
class ControllerParams:
    k_es_impact: float = 0.30
    k_es_stance: float = 0.30
    k_ls_slip: float = 0.30
    k_ls_stance: float = 0.30
    k_trip_impact: float = 0.30
    k_trip_stance: float = 0.30

    angle_gain_per_deg: float = 0.02

    trip_com_scaling_enable: bool = False
    trip_com_gain_per_mps: float = 1.0
    manual_mean_com_velocity_mps: float = 0.8

    right_flexion_sign: int = +1
    left_flexion_sign: int = +1
    right_torque_sign: int = +1
    left_torque_sign: int = +1

    enable_torque_clamp: bool = False
    torque_limit_nm: float = 10.0

    zero_cross_threshold_dps: float = 5.0
    max_active_ms: float = 700.0
    trip_foot_contact_delay_ms: float = 220.0


COLUMN_ALIASES = {
    "time": [
        "time", "time_s", "t", "timestamp", "elapsed_s", "elapsed_time"
    ],
    "rh_angle": [
        "right_hip_angle_deg", "rh_angle", "rightHipAngle",
        "right_hip_angle", "hip_angle_r", "R_HipAngle"
    ],
    "lh_angle": [
        "left_hip_angle_deg", "lh_angle", "leftHipAngle",
        "left_hip_angle", "hip_angle_l", "L_HipAngle"
    ],
    "rh_qdot": [
        "right_hip_vel_dps", "rh_qdot", "right_hip_velocity",
        "rightHipVelocity", "hip_velocity_r", "R_HipVel"
    ],
    "lh_qdot": [
        "left_hip_vel_dps", "lh_qdot", "left_hip_velocity",
        "leftHipVelocity", "hip_velocity_l", "L_HipVel"
    ],
    "com_v": [
        "com_velocity_mps", "com_vel", "COM_velocity",
        "forward_com_velocity", "com_forward_velocity"
    ],
}


def find_column(df: pd.DataFrame, key: str):
    lower_map = {str(c).lower(): c for c in df.columns}
    for alias in COLUMN_ALIASES[key]:
        if alias.lower() in lower_map:
            return lower_map[alias.lower()]
    return None


def finite_difference_deg(angle_deg: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    if len(angle_deg) < 2:
        return np.zeros_like(angle_deg, dtype=float)
    return np.gradient(angle_deg.astype(float), time_s.astype(float))


def synthetic_data():
    """
    Synthetic data is NOT a biomechanics model.
    It is only a convenient waveform for coefficient preview.
    """
    t = np.linspace(-0.25, 0.90, 1151)

    # Angles: smooth illustrative trajectories.
    rh_angle = 10.0 + 18.0 * np.sin(2.0 * np.pi * (t + 0.05))
    lh_angle = 5.0 + 16.0 * np.sin(2.0 * np.pi * (t + 0.55))

    # Add a transient perturbation-shaped velocity excursion after detection t=0.
    rh_qdot = np.gradient(rh_angle, t)
    lh_qdot = np.gradient(lh_angle, t)

    pulse = np.exp(-((t - 0.12) / 0.085) ** 2)
    rh_qdot += 170.0 * pulse
    lh_qdot -= 120.0 * np.exp(-((t - 0.15) / 0.11) ** 2)

    com_v = 0.8 + 0.18 * np.exp(-((t - 0.08) / 0.12) ** 2)

    return {
        "time": t,
        "rh_angle": rh_angle,
        "lh_angle": lh_angle,
        "rh_qdot": rh_qdot,
        "lh_qdot": lh_qdot,
        "com_v": com_v,
        "source": "synthetic preview",
    }


def simulate_controller(
    data: dict,
    event: str,
    side: str,
    p: ControllerParams,
):
    t = np.asarray(data["time"], dtype=float)
    rh_angle = np.asarray(data["rh_angle"], dtype=float)
    lh_angle = np.asarray(data["lh_angle"], dtype=float)
    rh_qdot = np.asarray(data["rh_qdot"], dtype=float)
    lh_qdot = np.asarray(data["lh_qdot"], dtype=float)

    rh_flex = p.right_flexion_sign * rh_qdot
    lh_flex = p.left_flexion_sign * lh_qdot

    angle_diff = np.abs(rh_angle - lh_angle)
    angle_scale = 1.0 + p.angle_gain_per_deg * angle_diff

    rh_tau = np.zeros_like(t)
    lh_tau = np.zeros_like(t)
    rh_B = np.zeros_like(t)
    lh_B = np.zeros_like(t)

    detect_idx_candidates = np.where(t >= 0.0)[0]
    if len(detect_idx_candidates) == 0:
        detect_idx = 0
    else:
        detect_idx = int(detect_idx_candidates[0])

    if event == "Early-swing trip":
        active_time = p.trip_foot_contact_delay_ms / 1000.0
    else:
        active_time = 0.0

    active_candidates = np.where(t >= active_time)[0]
    active_idx = int(active_candidates[0]) if len(active_candidates) else len(t) - 1

    # Trip mean COM velocity between obstacle contact (t=0) and foot contact.
    if event == "Early-swing trip":
        if data.get("com_v") is not None:
            com_v = np.asarray(data["com_v"], dtype=float)
            region = (t >= 0.0) & (t <= active_time)
            if np.any(region):
                mean_com_v = float(np.mean(com_v[region]))
            else:
                mean_com_v = p.manual_mean_com_velocity_mps
        else:
            mean_com_v = p.manual_mean_com_velocity_mps
    else:
        mean_com_v = 0.0

    if p.trip_com_scaling_enable:
        com_scale = 1.0 + p.trip_com_gain_per_mps * abs(mean_com_v)
    else:
        com_scale = 1.0

    primary_is_right = side == "Right"
    primary_flex = rh_flex if primary_is_right else lh_flex

    if event in ("ES slip", "Early-swing trip"):
        harmful_dir = +1
    else:
        harmful_dir = -1

    stop_idx = len(t)
    max_end_time = active_time + p.max_active_ms / 1000.0

    for i in range(active_idx, len(t)):
        if t[i] >= max_end_time:
            stop_idx = i
            break

        # Stop on clear primary velocity reversal.
        if harmful_dir > 0:
            reversed_now = primary_flex[i] <= -p.zero_cross_threshold_dps
        else:
            reversed_now = primary_flex[i] >= +p.zero_cross_threshold_dps

        # Avoid stopping at the very first active sample.
        if i > active_idx + 2 and reversed_now:
            stop_idx = i
            break

        if primary_is_right:
            p_qdot, s_qdot = rh_qdot[i], lh_qdot[i]
            p_flex, s_flex = rh_flex[i], lh_flex[i]
        else:
            p_qdot, s_qdot = lh_qdot[i], rh_qdot[i]
            p_flex, s_flex = lh_flex[i], rh_flex[i]

        Bp = 0.0
        Bs = 0.0
        p_enable = False
        s_enable = False

        if event == "ES slip":
            Bp = p.k_es_impact
            Bs = p.k_es_stance * angle_scale[i]
            p_enable = p_flex > 0.0      # damp flexion
            s_enable = s_flex < 0.0      # damp extension

        elif event == "LS slip":
            Bp = p.k_ls_slip
            Bs = p.k_ls_stance * angle_scale[i]
            p_enable = p_flex < 0.0      # damp extension
            s_enable = s_flex > 0.0      # damp flexion

        elif event == "Early-swing trip":
            Bp = p.k_trip_impact * com_scale
            Bs = p.k_trip_stance
            p_enable = p_flex > 0.0      # damp flexion
            s_enable = s_flex < 0.0      # damp extension

        if p_enable:
            tau_p = -Bp * p_qdot * DEG2RAD
        else:
            tau_p = 0.0

        if s_enable:
            tau_s = -Bs * s_qdot * DEG2RAD
        else:
            tau_s = 0.0

        if primary_is_right:
            rh_B[i] = Bp if p_enable else 0.0
            lh_B[i] = Bs if s_enable else 0.0
            rh_tau[i] = p.right_torque_sign * tau_p
            lh_tau[i] = p.left_torque_sign * tau_s
        else:
            lh_B[i] = Bp if p_enable else 0.0
            rh_B[i] = Bs if s_enable else 0.0
            lh_tau[i] = p.left_torque_sign * tau_p
            rh_tau[i] = p.right_torque_sign * tau_s

    # Zero after stop.
    rh_tau[stop_idx:] = 0.0
    lh_tau[stop_idx:] = 0.0
    rh_B[stop_idx:] = 0.0
    lh_B[stop_idx:] = 0.0

    raw_rh = rh_tau.copy()
    raw_lh = lh_tau.copy()

    if p.enable_torque_clamp:
        lim = abs(p.torque_limit_nm)
        rh_tau = np.clip(rh_tau, -lim, lim)
        lh_tau = np.clip(lh_tau, -lim, lim)

    return {
        "time": t,
        "rh_qdot": rh_qdot,
        "lh_qdot": lh_qdot,
        "rh_angle": rh_angle,
        "lh_angle": lh_angle,
        "angle_scale": angle_scale,
        "rh_B": rh_B,
        "lh_B": lh_B,
        "rh_tau_raw": raw_rh,
        "lh_tau_raw": raw_lh,
        "rh_tau": rh_tau,
        "lh_tau": lh_tau,
        "detect_time": 0.0,
        "active_time": active_time,
        "stop_time": t[min(stop_idx, len(t)-1)],
        "mean_com_v": mean_com_v,
        "com_scale": com_scale,
    }


class DampingTuningGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("H10 Slip / Trip Damping Tuning Preview")
        self.root.geometry("1320x860")

        self.data = synthetic_data()
        self.params = ControllerParams()

        self.event_var = tk.StringVar(value="ES slip")
        self.side_var = tk.StringVar(value="Right")

        self.k_es_impact = tk.DoubleVar(value=self.params.k_es_impact)
        self.k_es_stance = tk.DoubleVar(value=self.params.k_es_stance)
        self.k_ls_slip = tk.DoubleVar(value=self.params.k_ls_slip)
        self.k_ls_stance = tk.DoubleVar(value=self.params.k_ls_stance)
        self.k_trip_impact = tk.DoubleVar(value=self.params.k_trip_impact)
        self.k_trip_stance = tk.DoubleVar(value=self.params.k_trip_stance)

        self.angle_gain = tk.DoubleVar(value=self.params.angle_gain_per_deg)
        self.trip_com_enable = tk.BooleanVar(value=False)
        self.trip_com_gain = tk.DoubleVar(value=self.params.trip_com_gain_per_mps)
        self.manual_com = tk.DoubleVar(value=self.params.manual_mean_com_velocity_mps)

        self.zero_cross = tk.DoubleVar(value=self.params.zero_cross_threshold_dps)
        self.max_active = tk.DoubleVar(value=self.params.max_active_ms)
        self.trip_delay = tk.DoubleVar(value=self.params.trip_foot_contact_delay_ms)

        self.clamp_enable = tk.BooleanVar(value=False)
        self.torque_limit = tk.DoubleVar(value=self.params.torque_limit_nm)

        self.status_var = tk.StringVar(value="Synthetic preview loaded.")

        self._build_ui()
        self.update_plot()

    def _build_ui(self):
        left = ttk.Frame(self.root, padding=10)
        left.pack(side=tk.LEFT, fill=tk.Y)

        right = ttk.Frame(self.root, padding=8)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        ttk.Label(left, text="Event").pack(anchor="w")
        event_box = ttk.Combobox(
            left,
            textvariable=self.event_var,
            state="readonly",
            values=["ES slip", "LS slip", "Early-swing trip"],
            width=23,
        )
        event_box.pack(fill=tk.X, pady=(0, 6))
        event_box.bind("<<ComboboxSelected>>", lambda _: self.update_plot())

        ttk.Label(left, text="Primary / perturbed side").pack(anchor="w")
        side_box = ttk.Combobox(
            left,
            textvariable=self.side_var,
            state="readonly",
            values=["Right", "Left"],
            width=23,
        )
        side_box.pack(fill=tk.X, pady=(0, 10))
        side_box.bind("<<ComboboxSelected>>", lambda _: self.update_plot())

        ttk.Button(left, text="Load CSV", command=self.load_csv).pack(fill=tk.X)
        ttk.Button(left, text="Use synthetic preview", command=self.use_synthetic).pack(fill=tk.X, pady=(4, 10))

        self._section(left, "Damping coefficients [Nm·s/rad]")
        self._slider(left, "ES impact", self.k_es_impact, 0.0, 3.0, 0.01)
        self._slider(left, "ES stance", self.k_es_stance, 0.0, 3.0, 0.01)
        self._slider(left, "LS slipping", self.k_ls_slip, 0.0, 3.0, 0.01)
        self._slider(left, "LS stance", self.k_ls_stance, 0.0, 3.0, 0.01)
        self._slider(left, "Trip impact", self.k_trip_impact, 0.0, 3.0, 0.01)
        self._slider(left, "Trip stance", self.k_trip_stance, 0.0, 3.0, 0.01)

        self._section(left, "Scaling")
        self._slider(left, "Angle gain [/deg]", self.angle_gain, 0.0, 0.10, 0.001)

        ttk.Checkbutton(
            left,
            text="Enable trip COM scaling",
            variable=self.trip_com_enable,
            command=self.update_plot,
        ).pack(anchor="w")

        self._slider(left, "COM gain [s/m]", self.trip_com_gain, 0.0, 5.0, 0.05)
        self._slider(left, "Manual mean COM v [m/s]", self.manual_com, 0.0, 3.0, 0.02)

        self._section(left, "Timing / stop")
        self._slider(left, "Trip foot-contact delay [ms]", self.trip_delay, 0, 600, 5)
        self._slider(left, "Zero-cross threshold [deg/s]", self.zero_cross, 0, 30, 1)
        self._slider(left, "Max active [ms]", self.max_active, 100, 1500, 10)

        self._section(left, "Torque clamp preview")
        ttk.Checkbutton(
            left,
            text="Enable clamp in preview",
            variable=self.clamp_enable,
            command=self.update_plot,
        ).pack(anchor="w")
        self._slider(left, "Torque limit [Nm]", self.torque_limit, 0.5, 20.0, 0.1)

        ttk.Button(left, text="Copy C parameters", command=self.copy_c_parameters).pack(fill=tk.X, pady=(10, 4))
        ttk.Label(left, textvariable=self.status_var, wraplength=330).pack(anchor="w", pady=(8, 0))

        self.fig = Figure(figsize=(9.0, 7.5), dpi=100)
        self.ax_vel = self.fig.add_subplot(311)
        self.ax_tau = self.fig.add_subplot(312)
        self.ax_scale = self.fig.add_subplot(313)

        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _section(self, parent, text):
        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(10, 5))
        ttk.Label(parent, text=text).pack(anchor="w")

    def _slider(self, parent, label, var, vmin, vmax, resolution):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=1)

        ttk.Label(frame, text=label, width=27).pack(side=tk.LEFT)

        scale = tk.Scale(
            frame,
            variable=var,
            from_=vmin,
            to=vmax,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            length=170,
            showvalue=True,
            command=lambda _value: self.update_plot(),
        )
        scale.pack(side=tk.RIGHT)

    def current_params(self):
        return ControllerParams(
            k_es_impact=float(self.k_es_impact.get()),
            k_es_stance=float(self.k_es_stance.get()),
            k_ls_slip=float(self.k_ls_slip.get()),
            k_ls_stance=float(self.k_ls_stance.get()),
            k_trip_impact=float(self.k_trip_impact.get()),
            k_trip_stance=float(self.k_trip_stance.get()),
            angle_gain_per_deg=float(self.angle_gain.get()),
            trip_com_scaling_enable=bool(self.trip_com_enable.get()),
            trip_com_gain_per_mps=float(self.trip_com_gain.get()),
            manual_mean_com_velocity_mps=float(self.manual_com.get()),
            enable_torque_clamp=bool(self.clamp_enable.get()),
            torque_limit_nm=float(self.torque_limit.get()),
            zero_cross_threshold_dps=float(self.zero_cross.get()),
            max_active_ms=float(self.max_active.get()),
            trip_foot_contact_delay_ms=float(self.trip_delay.get()),
        )

    def load_csv(self):
        path = filedialog.askopenfilename(
            title="Load gait / perturbation CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return

        try:
            df = pd.read_csv(path)
        except Exception as exc:
            messagebox.showerror("CSV error", str(exc))
            return

        time_col = find_column(df, "time")
        rh_angle_col = find_column(df, "rh_angle")
        lh_angle_col = find_column(df, "lh_angle")
        rh_qdot_col = find_column(df, "rh_qdot")
        lh_qdot_col = find_column(df, "lh_qdot")
        com_col = find_column(df, "com_v")

        if time_col is None:
            # Assume 1 kHz if no time column.
            time = np.arange(len(df), dtype=float) * 0.001
        else:
            time = pd.to_numeric(df[time_col], errors="coerce").to_numpy(dtype=float)

        # Shift so first sample is shown relative to zero.
        time = time - time[0]

        if rh_angle_col is None or lh_angle_col is None:
            messagebox.showerror(
                "Missing columns",
                "CSV needs left and right hip-angle columns.\n"
                "Supported examples: right_hip_angle_deg, left_hip_angle_deg."
            )
            return

        rh_angle = pd.to_numeric(df[rh_angle_col], errors="coerce").interpolate().bfill().ffill().to_numpy()
        lh_angle = pd.to_numeric(df[lh_angle_col], errors="coerce").interpolate().bfill().ffill().to_numpy()

        if rh_qdot_col is None:
            rh_qdot = finite_difference_deg(rh_angle, time)
        else:
            rh_qdot = pd.to_numeric(df[rh_qdot_col], errors="coerce").interpolate().bfill().ffill().to_numpy()

        if lh_qdot_col is None:
            lh_qdot = finite_difference_deg(lh_angle, time)
        else:
            lh_qdot = pd.to_numeric(df[lh_qdot_col], errors="coerce").interpolate().bfill().ffill().to_numpy()

        if com_col is None:
            com_v = None
        else:
            com_v = pd.to_numeric(df[com_col], errors="coerce").interpolate().bfill().ffill().to_numpy()

        # Use the first sample as detection t=0 for a simple preview.
        self.data = {
            "time": time,
            "rh_angle": rh_angle,
            "lh_angle": lh_angle,
            "rh_qdot": rh_qdot,
            "lh_qdot": lh_qdot,
            "com_v": com_v,
            "source": path,
        }

        self.status_var.set(
            f"Loaded {path}\n"
            f"RH qdot: {'CSV' if rh_qdot_col else 'differentiated from angle'}, "
            f"LH qdot: {'CSV' if lh_qdot_col else 'differentiated from angle'}, "
            f"COM: {'CSV' if com_col else 'manual/no COM column'}"
        )
        self.update_plot()

    def use_synthetic(self):
        self.data = synthetic_data()
        self.status_var.set("Synthetic preview loaded.")
        self.update_plot()

    def update_plot(self):
        if not hasattr(self, "ax_vel"):
            return

        p = self.current_params()
        result = simulate_controller(
            self.data,
            self.event_var.get(),
            self.side_var.get(),
            p,
        )

        self.ax_vel.clear()
        self.ax_tau.clear()
        self.ax_scale.clear()

        t = result["time"]

        self.ax_vel.plot(t, result["rh_qdot"], label="RH qdot")
        self.ax_vel.plot(t, result["lh_qdot"], label="LH qdot")
        self.ax_vel.axvline(result["detect_time"], linestyle="--", label="detector")
        if self.event_var.get() == "Early-swing trip":
            self.ax_vel.axvline(result["active_time"], linestyle=":", label="trip foot contact")
        self.ax_vel.axvline(result["stop_time"], linestyle="-.", label="controller stop")
        self.ax_vel.set_ylabel("Hip qdot [deg/s]")
        self.ax_vel.legend(loc="upper right")
        self.ax_vel.grid(True, alpha=0.2)

        self.ax_tau.plot(t, result["rh_tau_raw"], label="RH raw torque")
        self.ax_tau.plot(t, result["lh_tau_raw"], label="LH raw torque")

        if p.enable_torque_clamp:
            self.ax_tau.plot(t, result["rh_tau"], linestyle="--", label="RH clamped")
            self.ax_tau.plot(t, result["lh_tau"], linestyle="--", label="LH clamped")

        self.ax_tau.axhline(0.0, linewidth=0.8)
        self.ax_tau.set_ylabel("Torque [Nm]")
        self.ax_tau.legend(loc="upper right")
        self.ax_tau.grid(True, alpha=0.2)

        self.ax_scale.plot(t, result["angle_scale"], label="inter-leg angle scale")
        self.ax_scale.plot(t, result["rh_B"], label="RH B_eff")
        self.ax_scale.plot(t, result["lh_B"], label="LH B_eff")
        self.ax_scale.set_xlabel("Time [s]")
        self.ax_scale.set_ylabel("Scale / B_eff")
        self.ax_scale.legend(loc="upper right")
        self.ax_scale.grid(True, alpha=0.2)

        rh_peak = float(np.max(np.abs(result["rh_tau_raw"])))
        lh_peak = float(np.max(np.abs(result["lh_tau_raw"])))

        title = (
            f"{self.event_var.get()} | {self.side_var.get()} primary | "
            f"raw peak torque RH={rh_peak:.2f} Nm, LH={lh_peak:.2f} Nm"
        )
        if self.event_var.get() == "Early-swing trip":
            title += (
                f" | mean COM v={result['mean_com_v']:.3f} m/s"
                f" | COM scale={result['com_scale']:.2f}"
            )

        self.fig.suptitle(title)
        self.fig.tight_layout(rect=[0, 0, 1, 0.96])
        self.canvas.draw_idle()

    def copy_c_parameters(self):
        p = self.current_params()
        text = f"""/* Paste/enter these values into CubeIDE Live Expressions */
g_k_es_impact = {p.k_es_impact:.4f}f;
g_k_es_stance = {p.k_es_stance:.4f}f;
g_k_ls_slip = {p.k_ls_slip:.4f}f;
g_k_ls_stance = {p.k_ls_stance:.4f}f;
g_k_trip_impact = {p.k_trip_impact:.4f}f;
g_k_trip_stance = {p.k_trip_stance:.4f}f;
g_angle_gain_per_deg = {p.angle_gain_per_deg:.5f}f;
g_trip_com_scaling_enable = {1 if p.trip_com_scaling_enable else 0}U;
g_trip_com_gain_per_mps = {p.trip_com_gain_per_mps:.4f}f;
g_zero_cross_threshold_dps = {p.zero_cross_threshold_dps:.2f}f;
g_max_active_ms = {(int(p.max_active_ms))}U;
g_trip_fallback_delay_ms = {(int(p.trip_foot_contact_delay_ms))}U;
g_enable_torque_clamp = {1 if p.enable_torque_clamp else 0}U;
g_torque_limit_nm = {p.torque_limit_nm:.2f}f;
"""
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.status_var.set("Current parameters copied to clipboard as C assignments.")


def main():
    root = tk.Tk()
    app = DampingTuningGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
