#!/usr/bin/env python3
"""Focused live logger for firmware/damping_assist.c (USB-CDC damping stream)."""

from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import re
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Any

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore[assignment]

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

from xm_daq_protocol import (
    DEBUG_MODULE_ID,
    ROBOT_MODULE_ID,
    PhaiStreamDecoder,
    ProtocolError,
    build_test_packet,
)


APP_TITLE = "IFAC Damping Assist Logger"
DEFAULT_BAUD = 921600
EXPECTED_PROTOCOL_VERSION = 1.0
TORQUE_VELOCITY_LPF_HZ = 4.0

# Keep this list in exactly the same order as DAMPING_STREAM_CHANNELS in firmware.
DAMPING_LABELS = [
    "protocol_version",
    "build_id",
    "mcu_time_ms",
    "robot_frame_sequence",
    "user_loop_count",
    "usb_tx_fail_count",
    "loop_tick_gap_count",
    "usb_connected",
    "usb_streaming_active",
    "h10_is_connected",
    "h10_mode",
    "h10_assist_level",
    "h10_fsm_state",
    "xm_control_mode",
    "control_on",
    "torque_clamp_enabled",
    "torque_limit_nm",
    "manual_event_side",
    "button1_state",
    "button2_state",
    "button3_state",
    "marker_sequence",
    "marker_code",
    "marker_kind",
    "marker_tick_ms",
    "marker_accepted",
    "marker_event_type",
    "marker_event_side",
    "manual_trigger_count",
    "active_event_type",
    "active_event_side",
    "reaction_state",
    "threshold_first_crossed",
    "reaction_stop_count",
    "last_stop_flex_vel_dps",
    "trip_waiting_for_contact",
    "trip_seen_airborne",
    "left_foot_contact",
    "right_foot_contact",
    "left_hip_angle_deg",
    "right_hip_angle_deg",
    "left_hip_velocity_dps",
    "right_hip_velocity_dps",
    "left_flex_velocity_dps",
    "right_flex_velocity_dps",
    "interleg_angle_deg",
    "left_b_eff_nms_rad",
    "right_b_eff_nms_rad",
    "left_tau_raw_nm",
    "right_tau_raw_nm",
    "left_tau_cmd_nm",
    "right_tau_cmd_nm",
    "left_motor_current_a",
    "right_motor_current_a",
    "left_joint_torque_est_nm",
    "right_joint_torque_est_nm",
    "angle_scale",
    "trip_mean_com_velocity_mps",
    "trip_com_scale",
    "k_es_impact_nms_rad",
    "k_es_stance_nms_rad",
    "k_ls_slip_nms_rad",
    "k_ls_stance_nms_rad",
    "k_trip_impact_nms_rad",
    "k_trip_stance_nms_rad",
    "angle_gain_per_deg",
    "trip_com_scaling_enabled",
    "trip_com_gain_per_mps",
    "reversal_threshold_dps",
    "velocity_lpf_hz",
]
DAMPING_PAYLOAD_SIZE = len(DAMPING_LABELS) * 4

EVENT_NAMES = {
    1: "es_slip",
    2: "ls_slip",
    3: "early_swing_trip",
}
SIDE_NAMES = {
    0: "none",
    1: "right",
    2: "left",
}
MARKER_KIND_NAMES = {
    1: "click",
    2: "long_press",
}


def iso_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def slug(value: str, fallback: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "-", value.strip()).strip("-._")
    return cleaned or fallback


def available_ports() -> list[tuple[str, str]]:
    if serial is None:
        return []
    return [
        (port.device, f"{port.device} - {port.description or 'serial device'}")
        for port in serial.tools.list_ports.comports()
    ]


def decode_damping_packet(packet: Any) -> dict[str, float]:
    if packet.module_id != ROBOT_MODULE_ID:
        raise ProtocolError(f"unexpected module 0x{packet.module_id:02X}")
    if len(packet.payload) != DAMPING_PAYLOAD_SIZE:
        raise ProtocolError(
            f"damping payload is {len(packet.payload)} bytes; "
            f"expected {DAMPING_PAYLOAD_SIZE}"
        )
    values = struct.unpack(f"<{len(DAMPING_LABELS)}f", packet.payload)
    row = dict(zip(DAMPING_LABELS, values, strict=True))
    if not math.isclose(
        row["protocol_version"], EXPECTED_PROTOCOL_VERSION, abs_tol=1e-6
    ):
        raise ProtocolError(
            f"unsupported damping protocol {row['protocol_version']:.3f}"
        )
    return row


def marker_description(row: dict[str, float]) -> tuple[str, str]:
    button = int(round(row["marker_code"]))
    kind_code = int(round(row["marker_kind"]))
    accepted = bool(round(row["marker_accepted"]))
    event_code = int(round(row["marker_event_type"]))
    side_code = int(round(row["marker_event_side"]))
    kind = MARKER_KIND_NAMES.get(kind_code, f"kind_{kind_code}")
    event = EVENT_NAMES.get(event_code, f"event_{event_code}")
    side = SIDE_NAMES.get(side_code, f"side_{side_code}")

    if kind_code == 2:
        label = f"button_{button}_long_press"
    elif accepted:
        label = f"{event}_trigger"
    else:
        label = f"{event}_click_rejected"

    detail = (
        f"button={button}; kind={kind}; accepted={int(accepted)}; "
        f"event={event}; side={side}; "
        f"xm_tick_ms={int(round(row['marker_tick_ms']))}; "
        f"sequence={int(round(row['marker_sequence']))}"
    )
    return label, detail


def button_event_text(row: dict[str, float]) -> str:
    button = int(round(row["marker_code"]))
    kind_code = int(round(row["marker_kind"]))
    accepted = bool(round(row["marker_accepted"]))
    event_code = int(round(row["marker_event_type"]))
    side_code = int(round(row["marker_event_side"]))
    event = EVENT_NAMES.get(event_code, f"event {event_code}").replace("_", " ")
    side = SIDE_NAMES.get(side_code, f"side {side_code}")

    if kind_code == 2:
        return f"Button {button} long press detected — no damping trigger."
    result = "accepted" if accepted else "rejected (controller not idle/active)"
    return (
        f"Button {button} pressed (short click/release) — "
        f"{event}, {side} side, trigger {result}."
    )


class SessionWriter:
    def __init__(self, output_root: Path, metadata: dict[str, Any]) -> None:
        self.metadata = dict(metadata)
        self.start_wall_iso = iso_now()
        self.start_monotonic = time.perf_counter()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        subject = slug(str(metadata["subject"]), "subject")
        condition = slug(str(metadata["condition"]), "damping")
        trial = slug(str(metadata["trial"]), "T00")
        session_dir = output_root / datetime.now().strftime("%Y%m%d") / subject
        session_dir.mkdir(parents=True, exist_ok=True)
        self.prefix = session_dir / f"{stamp}_{subject}_{condition}_{trial}"

        self.data_path = self.prefix.with_name(self.prefix.name + "_damping.csv")
        self.event_path = self.prefix.with_name(self.prefix.name + "_events.csv")
        self.raw_path = self.prefix.with_name(self.prefix.name + "_raw.phai.bin")
        self.metadata_path = self.prefix.with_name(
            self.prefix.name + "_metadata.json"
        )

        self._data_file = self.data_path.open("w", newline="", encoding="utf-8")
        self._event_file = self.event_path.open("w", newline="", encoding="utf-8")
        self._raw_file = self.raw_path.open("wb")
        self._data_writer = csv.DictWriter(
            self._data_file,
            fieldnames=[
                "host_time_iso",
                "host_elapsed_s",
                "phai_seq_id",
                "phai_status",
                *DAMPING_LABELS,
            ],
        )
        self._event_writer = csv.DictWriter(
            self._event_file,
            fieldnames=[
                "event_index",
                "host_time_iso",
                "host_elapsed_s",
                "mcu_time_ms",
                "label",
                "detail",
                "marker_sequence",
                "marker_code",
                "marker_kind",
                "marker_accepted",
                "marker_event_type",
                "marker_event_side",
                "active_event_type",
                "active_event_side",
                "reaction_state",
            ],
        )
        self._data_writer.writeheader()
        self._event_writer.writeheader()
        self.data_rows = 0
        self.event_rows = 0
        self.raw_bytes = 0
        self.latest_row: dict[str, float] = {}
        self._last_flush = time.perf_counter()
        self.add_event("logging_start", "")

    def elapsed(self) -> float:
        return time.perf_counter() - self.start_monotonic

    def write_raw(self, data: bytes) -> None:
        self._raw_file.write(data)
        self.raw_bytes += len(data)

    def write_data(self, row: dict[str, float], seq_id: int, status: int) -> None:
        record: dict[str, Any] = {
            "host_time_iso": iso_now(),
            "host_elapsed_s": f"{self.elapsed():.6f}",
            "phai_seq_id": seq_id,
            "phai_status": status,
            **row,
        }
        self._data_writer.writerow(record)
        self.data_rows += 1
        self.latest_row = dict(row)
        self._flush_periodically()

    def add_event(
        self, label: str, detail: str, row: dict[str, float] | None = None
    ) -> None:
        source = row or self.latest_row
        self.event_rows += 1
        self._event_writer.writerow(
            {
                "event_index": self.event_rows,
                "host_time_iso": iso_now(),
                "host_elapsed_s": f"{self.elapsed():.6f}",
                "mcu_time_ms": source.get("mcu_time_ms", ""),
                "label": label,
                "detail": detail,
                "marker_sequence": source.get("marker_sequence", ""),
                "marker_code": source.get("marker_code", ""),
                "marker_kind": source.get("marker_kind", ""),
                "marker_accepted": source.get("marker_accepted", ""),
                "marker_event_type": source.get("marker_event_type", ""),
                "marker_event_side": source.get("marker_event_side", ""),
                "active_event_type": source.get("active_event_type", ""),
                "active_event_side": source.get("active_event_side", ""),
                "reaction_state": source.get("reaction_state", ""),
            }
        )
        self._event_file.flush()

    def _flush_periodically(self) -> None:
        now = time.perf_counter()
        if now - self._last_flush >= 1.0:
            self._data_file.flush()
            self._event_file.flush()
            self._raw_file.flush()
            self._last_flush = now

    def close(self, results: dict[str, Any]) -> None:
        self.add_event("logging_stop", str(results.get("stop_reason", "")))
        self._data_file.flush()
        self._event_file.flush()
        self._raw_file.flush()
        self._data_file.close()
        self._event_file.close()
        self._raw_file.close()
        metadata = {
            **self.metadata,
            "start_wall_time": self.start_wall_iso,
            "stop_wall_time": iso_now(),
            "protocol_version": EXPECTED_PROTOCOL_VERSION,
            "damping_schema": DAMPING_LABELS,
            "packet_payload_bytes": DAMPING_PAYLOAD_SIZE,
            "files": {
                "damping_csv": self.data_path.name,
                "events_csv": self.event_path.name,
                "raw_phai": self.raw_path.name,
            },
            "results": {
                **results,
                "data_rows": self.data_rows,
                "event_rows": self.event_rows,
                "raw_bytes": self.raw_bytes,
            },
        }
        self.metadata_path.write_text(
            json.dumps(metadata, indent=2, allow_nan=False),
            encoding="utf-8",
        )


class SerialWorker(threading.Thread):
    def __init__(
        self, port: str, baud: int, messages: queue.Queue[dict[str, Any]]
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.messages = messages
        self.stop_requested = threading.Event()
        self.decoder = PhaiStreamDecoder()
        self._record_lock = threading.Lock()
        self._writer: SessionWriter | None = None
        self._plot_lock = threading.Lock()
        self._preview_started_monotonic = time.perf_counter()
        self._last_publish = 0.0
        self._last_damping_packet = 0.0
        self._previous_marker_sequence: int | None = None
        self._previous_stop_count: int | None = None
        self._previous_robot_sequence: int | None = None
        self._torque_filter_event: tuple[int, int, int] | None = None
        self._torque_filter_peak_reached = False
        self._torque_filter_peak_velocity = 0.0
        self._torque_filter_previous_nonzero: float | None = None
        self._torque_filter_right_velocity = 0.0
        self._torque_filter_left_velocity = 0.0
        self._torque_filter_previous_mcu_ms: float | None = None
        self.robot_drops = 0
        self.latest: dict[str, float] = {}
        self.plot_time: deque[float] = deque(maxlen=1200)
        self.plot_values = {
            name: deque(maxlen=1200)
            for name in (
                "left_hip_velocity_dps",
                "right_hip_velocity_dps",
                "left_flex_velocity_dps",
                "right_flex_velocity_dps",
                "left_hip_angle_deg",
                "right_hip_angle_deg",
                "left_b_eff_nms_rad",
                "right_b_eff_nms_rad",
                "left_tau_raw_nm",
                "right_tau_raw_nm",
                "left_tau_cmd_nm",
                "right_tau_cmd_nm",
                "active_event_type",
                "reaction_state",
                "button1_state",
                "button2_state",
                "button3_state",
                "left_torque_velocity_lpf_dps",
                "right_torque_velocity_lpf_dps",
                "sign_change_detected",
            )
        }

    def start_recording(self, writer: SessionWriter) -> bool:
        with self._record_lock:
            if self._writer is not None:
                return False
            self._writer = writer
            return True

    def stop_recording(self, reason: str) -> Path | None:
        with self._record_lock:
            writer = self._writer
            self._writer = None
        if writer is None:
            return None
        writer.close(
            {
                "stop_reason": reason,
                "robot_sequence_drops": self.robot_drops,
                "decoder_stats": vars(self.decoder.stats),
            }
        )
        return writer.prefix

    def is_recording(self) -> bool:
        with self._record_lock:
            return self._writer is not None

    def shutdown(self) -> None:
        self.stop_requested.set()

    def run(self) -> None:
        if serial is None:
            self.messages.put(
                {
                    "type": "error",
                    "message": (
                        "pyserial is required. Run: "
                        "python -m pip install -r requirements.txt"
                    ),
                }
            )
            self.messages.put({"type": "disconnected", "prefix": ""})
            return
        try:
            with serial.Serial(self.port, self.baud, timeout=0.05) as device:
                device.dtr = True
                device.write(b"AGRB MON START\r\n")
                device.flush()
                next_stream_retry = time.perf_counter() + 1.0
                self.messages.put({"type": "connected", "port": self.port})
                while not self.stop_requested.is_set():
                    chunk = device.read(8192)
                    if chunk:
                        with self._record_lock:
                            if self._writer is not None:
                                self._writer.write_raw(chunk)
                        for packet in self.decoder.feed(chunk):
                            self._process_packet(packet)
                    now = time.perf_counter()
                    if (
                        now >= next_stream_retry
                        and now - self._last_damping_packet >= 1.0
                    ):
                        device.write(b"AGRB MON START\r\n")
                        device.flush()
                        next_stream_retry = now + 1.0
                    self._publish_if_due()
        except Exception as exc:
            self.messages.put({"type": "error", "message": str(exc)})
        finally:
            prefix = self.stop_recording("serial_disconnect")
            self.messages.put(
                {
                    "type": "disconnected",
                    "prefix": str(prefix) if prefix else "",
                }
            )

    def _process_packet(self, packet: Any) -> None:
        if packet.module_id == ROBOT_MODULE_ID:
            try:
                row = decode_damping_packet(packet)
            except ProtocolError as exc:
                self.decoder.stats.framing_errors += 1
                self.messages.put({"type": "error", "message": str(exc)})
                return

            sequence = int(round(row["robot_frame_sequence"]))
            if (
                self._previous_robot_sequence is not None
                and sequence > self._previous_robot_sequence + 1
            ):
                self.robot_drops += sequence - self._previous_robot_sequence - 1
            self._previous_robot_sequence = sequence
            self.latest = row
            self._last_damping_packet = time.perf_counter()

            with self._record_lock:
                if self._writer is not None:
                    self._writer.write_data(row, packet.seq_id, packet.status)

            self._handle_marker(row)
            self._handle_reaction_stop(row)
            derived = self._update_torque_velocity_diagnostics(row)
            plot_row = {**row, **derived}
            with self._plot_lock:
                self.plot_time.append(
                    time.perf_counter() - self._preview_started_monotonic
                )
                for name, values in self.plot_values.items():
                    values.append(plot_row[name])
        elif packet.module_id == DEBUG_MODULE_ID:
            message = packet.payload.rstrip(b"\x00").decode(
                "utf-8", errors="replace"
            )
            if message:
                self.messages.put({"type": "debug", "message": message.rstrip()})

    def _handle_marker(self, row: dict[str, float]) -> None:
        sequence = int(round(row["marker_sequence"]))
        if self._previous_marker_sequence is None:
            self._previous_marker_sequence = sequence
            return
        if sequence == self._previous_marker_sequence:
            return
        self._previous_marker_sequence = sequence
        label, detail = marker_description(row)
        display = button_event_text(row)
        with self._record_lock:
            if self._writer is not None:
                self._writer.add_event(label, detail, row)
        self.messages.put(
            {
                "type": "event",
                "label": label,
                "detail": detail,
                "display": display,
            }
        )

    def _update_torque_velocity_diagnostics(
        self, row: dict[str, float]
    ) -> dict[str, float]:
        """Mirror the SJ torque-velocity path for live visualization only."""
        event_key = (
            int(round(row["active_event_type"])),
            int(round(row["active_event_side"])),
            int(round(row["reaction_state"])),
        )
        if event_key != self._torque_filter_event:
            self._torque_filter_event = event_key
            self._torque_filter_peak_reached = False
            self._torque_filter_peak_velocity = 0.0
            self._torque_filter_previous_nonzero = None

        mcu_ms = row["mcu_time_ms"]
        if self._torque_filter_previous_mcu_ms is None:
            dt_s = 0.0
        else:
            dt_s = max(0.0, (mcu_ms - self._torque_filter_previous_mcu_ms) * 0.001)
        self._torque_filter_previous_mcu_ms = mcu_ms

        right_velocity = row["right_flex_velocity_dps"]
        left_velocity = row["left_flex_velocity_dps"]
        event_type = event_key[0]
        primary_velocity = right_velocity if event_key[1] == 1 else left_velocity
        harmful_velocity = primary_velocity if event_type in (1, 3) else -primary_velocity

        if event_key[2] == 2 and event_type in (1, 2, 3):
            if not self._torque_filter_peak_reached:
                if harmful_velocity > self._torque_filter_peak_velocity:
                    self._torque_filter_peak_velocity = harmful_velocity
                elif (
                    self._torque_filter_peak_velocity > 0.0
                    and harmful_velocity < self._torque_filter_peak_velocity
                ):
                    self._torque_filter_peak_reached = True
                    self._torque_filter_previous_nonzero = None

            if dt_s > 0.0:
                alpha = (2.0 * math.pi * TORQUE_VELOCITY_LPF_HZ * dt_s) / (
                    1.0 + 2.0 * math.pi * TORQUE_VELOCITY_LPF_HZ * dt_s
                )
                self._torque_filter_right_velocity += alpha * (
                    right_velocity - self._torque_filter_right_velocity
                )
                self._torque_filter_left_velocity += alpha * (
                    left_velocity - self._torque_filter_left_velocity
                )

            sign_change = 0.0
            filtered_primary_velocity = (
                self._torque_filter_right_velocity
                if event_key[1] == 1
                else self._torque_filter_left_velocity
            )
            if (
                self._torque_filter_peak_reached
                and abs(filtered_primary_velocity) > 0.001
            ):
                if (
                    self._torque_filter_previous_nonzero is not None
                    and filtered_primary_velocity
                    * self._torque_filter_previous_nonzero
                    < 0.0
                ):
                    sign_change = filtered_primary_velocity
                self._torque_filter_previous_nonzero = filtered_primary_velocity
        else:
            sign_change = 0.0

        return {
            "left_torque_velocity_lpf_dps": (
                self._torque_filter_left_velocity
                if self._torque_filter_peak_reached
                else left_velocity
            ),
            "right_torque_velocity_lpf_dps": (
                self._torque_filter_right_velocity
                if self._torque_filter_peak_reached
                else right_velocity
            ),
            "sign_change_detected": sign_change,
        }

    def _handle_reaction_stop(self, row: dict[str, float]) -> None:
        count = int(round(row["reaction_stop_count"]))
        if self._previous_stop_count is None:
            self._previous_stop_count = count
            return
        if count == self._previous_stop_count:
            return
        delta = max(1, count - self._previous_stop_count)
        self._previous_stop_count = count
        detail = (
            f"count={count}; delta={delta}; "
            f"last_stop_flex_vel_dps={row['last_stop_flex_vel_dps']:.3f}"
        )
        with self._record_lock:
            if self._writer is not None:
                self._writer.add_event("reaction_stop", detail, row)
        self.messages.put(
            {"type": "event", "label": "reaction_stop", "detail": detail}
        )

    def _publish_if_due(self) -> None:
        now = time.perf_counter()
        if now - self._last_publish < 0.1:
            return
        self._last_publish = now
        with self._plot_lock:
            plot = {
                "time": list(self.plot_time),
                **{name: list(values) for name, values in self.plot_values.items()},
            }
        self.messages.put(
            {
                "type": "status",
                "latest": dict(self.latest),
                "plot": plot,
                "recording": self.is_recording(),
                "robot_drops": self.robot_drops,
                "decoder": vars(self.decoder.stats).copy(),
            }
        )


class LoggerApp:
    def __init__(self, root: tk.Tk, default_output: Path) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.messages: queue.Queue[dict[str, Any]] = queue.Queue()
        self.worker: SerialWorker | None = None
        self.port_lookup: dict[str, str] = {}
        self.plot_lines: dict[str, Any] = {}

        self.port_var = tk.StringVar()
        self.subject_var = tk.StringVar(value="S01")
        self.condition_var = tk.StringVar(value="mixed_damping")
        self.trial_var = tk.StringVar(value="T01")
        self.output_var = tk.StringVar(value=str(default_output))
        self.status_var = tk.StringVar(value="Disconnected")
        self.controller_var = tk.StringVar(value="No damping packet received")

        self._build_ui()
        self.refresh_ports()
        self.root.after(100, self._poll_messages)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(frame, text="Connection", padding=8)
        connection.pack(fill="x")
        self.port_combo = ttk.Combobox(
            connection, textvariable=self.port_var, width=42, state="readonly"
        )
        self.port_combo.pack(side="left", fill="x", expand=True)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).pack(
            side="left", padx=5
        )
        self.connect_button = ttk.Button(
            connection, text="Connect", command=self.connect
        )
        self.connect_button.pack(side="left")
        self.disconnect_button = ttk.Button(
            connection, text="Disconnect", command=self.disconnect, state="disabled"
        )
        self.disconnect_button.pack(side="left", padx=(5, 0))

        session = ttk.LabelFrame(frame, text="Focused damping session", padding=8)
        session.pack(fill="x", pady=(8, 0))
        for column, (label, variable, width) in enumerate(
            (
                ("Subject", self.subject_var, 12),
                ("Condition", self.condition_var, 22),
                ("Trial", self.trial_var, 8),
            )
        ):
            ttk.Label(session, text=label).grid(row=0, column=column * 2, sticky="w")
            ttk.Entry(session, textvariable=variable, width=width).grid(
                row=0, column=column * 2 + 1, padx=(4, 12)
            )
        ttk.Label(session, text="Output").grid(row=1, column=0, sticky="w", pady=(7, 0))
        ttk.Entry(session, textvariable=self.output_var).grid(
            row=1, column=1, columnspan=4, sticky="ew", pady=(7, 0)
        )
        ttk.Button(session, text="Browse", command=self.browse_output).grid(
            row=1, column=5, padx=(5, 0), pady=(7, 0)
        )
        session.columnconfigure(4, weight=1)

        actions = ttk.Frame(frame)
        actions.pack(fill="x", pady=8)
        self.start_button = ttk.Button(
            actions, text="Start logging", command=self.start_logging, state="disabled"
        )
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(
            actions, text="Stop", command=self.stop_logging, state="disabled"
        )
        self.stop_button.pack(side="left", padx=5)
        ttk.Label(actions, textvariable=self.status_var).pack(side="left", padx=12)

        ttk.Label(frame, textvariable=self.controller_var).pack(fill="x")

        if HAS_MATPLOTLIB:
            figure = Figure(figsize=(11, 9), dpi=100)
            self.axes = figure.subplots(3, 2)
            plot_specs = (
                (
                    self.axes[0][0],
                    (
                        ("left_hip_velocity_dps", "Left"),
                        ("right_hip_velocity_dps", "Right"),
                    ),
                    "Hip velocity [deg/s]",
                ),
                (
                    self.axes[0][1],
                    (
                        ("left_hip_angle_deg", "Left"),
                        ("right_hip_angle_deg", "Right"),
                    ),
                    "Hip angle [deg]",
                ),
                (
                    self.axes[1][0],
                    (
                        ("left_b_eff_nms_rad", "Left"),
                        ("right_b_eff_nms_rad", "Right"),
                    ),
                    "Effective damping [Nm s/rad]",
                ),
                (
                    self.axes[1][1],
                    (
                        ("left_tau_cmd_nm", "Left cmd"),
                        ("right_tau_cmd_nm", "Right cmd"),
                        ("left_tau_raw_nm", "Left raw"),
                        ("right_tau_raw_nm", "Right raw"),
                    ),
                    "Torque [Nm]",
                ),
                (
                    self.axes[2][0],
                    (
                        ("left_flex_velocity_dps", "Left flex raw"),
                        ("right_flex_velocity_dps", "Right flex raw"),
                        ("left_torque_velocity_lpf_dps", "Left 4 Hz LPF"),
                        ("right_torque_velocity_lpf_dps", "Right 4 Hz LPF"),
                    ),
                    "Torque velocity path [deg/s]",
                ),
                (
                    self.axes[2][1],
                    (
                        ("button1_state", "Button 1"),
                        ("button2_state", "Button 2"),
                        ("button3_state", "Button 3"),
                    ),
                    "Physical button state",
                ),
            )
            for axis, lines, ylabel in plot_specs:
                for name, label in lines:
                    (line,) = axis.plot([], [], label=label)
                    self.plot_lines[name] = line
                axis.set_ylabel(ylabel)
                axis.set_xlabel("Preview time [s]")
                if axis is self.axes[2][0]:
                    axis.axhline(0.0, color="black", linewidth=0.7)
                    (line,) = axis.plot([], [], "ko", markersize=4, label="sign change")
                    self.plot_lines["sign_change_detected"] = line
                axis.grid(True, alpha=0.3)
                axis.legend(loc="upper right", fontsize=8)
            figure.tight_layout()
            self.canvas = FigureCanvasTkAgg(figure, master=frame)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
        else:
            ttk.Label(
                frame,
                text="Install matplotlib for live plots; logging remains available.",
            ).pack(pady=20)

        events = ttk.LabelFrame(frame, text="Damping events", padding=5)
        events.pack(fill="both", pady=(8, 0))
        self.event_text = tk.Text(events, height=6, state="disabled")
        self.event_text.pack(fill="both", expand=True)

    def refresh_ports(self) -> None:
        ports = available_ports()
        self.port_lookup = {display: device for device, display in ports}
        displays = list(self.port_lookup)
        self.port_combo["values"] = displays
        if displays and self.port_var.get() not in displays:
            self.port_var.set(displays[0])

    def browse_output(self) -> None:
        selected = filedialog.askdirectory(initialdir=self.output_var.get())
        if selected:
            self.output_var.set(selected)

    def connect(self) -> None:
        if serial is None:
            messagebox.showerror(
                APP_TITLE,
                "pyserial is required. Run: python -m pip install -r requirements.txt",
            )
            return
        display = self.port_var.get()
        port = self.port_lookup.get(display, display)
        if not port:
            messagebox.showerror(APP_TITLE, "Select a serial port.")
            return
        self.worker = SerialWorker(port, DEFAULT_BAUD, self.messages)
        self.worker.start()
        self.status_var.set(f"Connecting to {port}...")
        self.connect_button.configure(state="disabled")

    def disconnect(self) -> None:
        if self.worker is not None:
            self.worker.shutdown()
        self.status_var.set("Disconnecting...")

    def start_logging(self) -> None:
        if self.worker is None or not self.worker.is_alive():
            messagebox.showerror(APP_TITLE, "Connect to the XM10 first.")
            return
        metadata = {
            "logger": Path(__file__).name,
            "subject": self.subject_var.get(),
            "condition": self.condition_var.get(),
            "trial": self.trial_var.get(),
            "serial_port": self.worker.port,
            "baud": self.worker.baud,
        }
        try:
            writer = SessionWriter(Path(self.output_var.get()), metadata)
        except Exception as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        if not self.worker.start_recording(writer):
            writer.close({"stop_reason": "start_rejected"})
            return
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status_var.set(f"Recording: {writer.prefix.name}")
        self._log_event("logging_start", str(writer.prefix))

    def stop_logging(self) -> None:
        if self.worker is None:
            return
        prefix = self.worker.stop_recording("user_stop")
        self.stop_button.configure(state="disabled")
        self.start_button.configure(state="normal")
        self.status_var.set("Connected - previewing")
        if prefix:
            self._log_event("logging_stop", str(prefix))

    def _poll_messages(self) -> None:
        try:
            while True:
                message = self.messages.get_nowait()
                kind = message["type"]
                if kind == "connected":
                    self.status_var.set(f"Connected: {message['port']} - previewing")
                    self.disconnect_button.configure(state="normal")
                    self.start_button.configure(state="normal")
                elif kind == "disconnected":
                    self.status_var.set("Disconnected")
                    self.connect_button.configure(state="normal")
                    self.disconnect_button.configure(state="disabled")
                    self.start_button.configure(state="disabled")
                    self.stop_button.configure(state="disabled")
                    self.worker = None
                elif kind == "error":
                    self._log_event("error", message["message"])
                elif kind == "debug":
                    self._log_event("firmware", message["message"])
                elif kind == "event":
                    self._log_event(
                        "BUTTON PRESS" if "display" in message else message["label"],
                        message.get("display", message["detail"]),
                    )
                elif kind == "status":
                    self._update_status(message)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_messages)

    def _update_status(self, message: dict[str, Any]) -> None:
        row = message["latest"]
        if row:
            event = EVENT_NAMES.get(
                int(round(row["active_event_type"])), "none"
            )
            side = SIDE_NAMES.get(int(round(row["active_event_side"])), "none")
            self.controller_var.set(
                f"H10 mode {int(round(row['h10_mode']))} | "
                f"control ON {int(round(row['control_on']))} | "
                f"event {event}/{side} | state {int(round(row['reaction_state']))} | "
                f"USB TX fails {int(round(row['usb_tx_fail_count']))} | "
                f"host frame drops {message['robot_drops']}"
            )
        if HAS_MATPLOTLIB:
            plot = message["plot"]
            times = plot["time"]
            for name, line in self.plot_lines.items():
                line.set_data(times, plot[name])
            for axis_row in self.axes:
                for axis in axis_row:
                    axis.relim()
                    axis.autoscale_view()
            self.canvas.draw_idle()

    def _log_event(self, label: str, detail: str) -> None:
        self.event_text.configure(state="normal")
        self.event_text.insert("end", f"{datetime.now():%H:%M:%S} {label}: {detail}\n")
        self.event_text.see("end")
        self.event_text.configure(state="disabled")

    def _on_close(self) -> None:
        if self.worker is not None:
            self.worker.stop_recording("window_close")
            self.worker.shutdown()
            self.worker.join(timeout=1.0)
        self.root.destroy()


def run_self_test() -> None:
    assert len(DAMPING_LABELS) == 70
    values = [float(index) for index in range(len(DAMPING_LABELS))]
    values[DAMPING_LABELS.index("protocol_version")] = EXPECTED_PROTOCOL_VERSION
    payload = struct.pack(f"<{len(values)}f", *values)
    encoded = build_test_packet(payload, ROBOT_MODULE_ID, seq_id=17)
    decoder = PhaiStreamDecoder()
    packets = []
    for offset in range(0, len(encoded), 7):
        packets.extend(decoder.feed(encoded[offset : offset + 7]))
    assert len(packets) == 1
    row = decode_damping_packet(packets[0])
    assert row["velocity_lpf_hz"] == values[-1]

    marker_row = dict(row)
    marker_row.update(
        {
            "marker_code": 3.0,
            "marker_kind": 1.0,
            "marker_accepted": 1.0,
            "marker_event_type": 3.0,
            "marker_event_side": 2.0,
            "marker_tick_ms": 1234.0,
            "marker_sequence": 9.0,
        }
    )
    label, detail = marker_description(marker_row)
    assert label == "early_swing_trip_trigger"
    assert "side=left" in detail
    print(
        f"PASS: {len(DAMPING_LABELS)} fields, "
        f"{DAMPING_PAYLOAD_SIZE}-byte payload, fragmented PhAI decode, markers"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent / "data",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        run_self_test()
        return
    root = tk.Tk()
    LoggerApp(root, args.output)
    root.mainloop()


if __name__ == "__main__":
    main()
