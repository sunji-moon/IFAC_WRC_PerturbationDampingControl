"""Protocol decoder shared by the IFAC XM logger and its offline tools."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable


PHAI_SOF = 0xAA
PHAI_HEADER_SIZE = 6
PHAI_CRC_SIZE = 2
ROBOT_MODULE_ID = 0xF0
EMG_MODULE_ID = 0xF2
DEBUG_MODULE_ID = 0xFF
EMG_MAGIC = 0x38474D45
EMG_PACKET_VERSION = 2
EMG_CHANNEL_COUNT = 8
EMG_CHANNEL_MASK = (1 << EMG_CHANNEL_COUNT) - 1
LEGACY_EMG_MAGIC = 0x34474D45
LEGACY_EMG_PACKET_VERSION = 1
LEGACY_EMG_CHANNEL_COUNT = 4
EMG_SAMPLES_PER_BLOCK = 10
EMG_HEADER_FORMAT = "<IHHIIIHHHH"
EMG_HEADER_SIZE = struct.calcsize(EMG_HEADER_FORMAT)
EMG_PAYLOAD_SIZE = EMG_HEADER_SIZE + 2 * EMG_CHANNEL_COUNT * EMG_SAMPLES_PER_BLOCK


# Firmware protocol 1.1, retained for live compatibility with older XM images.
LEGACY_ROBOT_LABELS = [
    "protocol_version",
    "build_id",
    "mcu_time_ms",
    "robot_frame_sequence",
    "user_loop_count",
    "usb_tx_fail_count",
    "emg_block_drop_count",
    "loop_tick_gap_count",
    "emg_adc_ready_mask",
    "h10_is_connected",
    "h10_mode",
    "h10_assist_level",
    "h10_fsm_state",
    "h10_assist_loop_count",
    "h10_postprocess_count",
    "h10_pvector_rh_done",
    "h10_pvector_lh_done",
    "h10_neutral_pos_set",
    "h10_left_foot_contact",
    "h10_right_foot_contact",
    "h10_forward_velocity_mps",
    "left_hip_angle_deg",
    "right_hip_angle_deg",
    "left_thigh_angle_deg",
    "right_thigh_angle_deg",
    "left_knee_angle_deg",
    "right_knee_angle_deg",
    "pelvic_angle_deg",
    "left_hip_motor_angle_deg",
    "right_hip_motor_angle_deg",
    "left_hip_motor_current_a",
    "right_hip_motor_current_a",
    "left_joint_torque_est_nm",
    "right_joint_torque_est_nm",
    "left_imu_roll_deg",
    "right_imu_roll_deg",
    "left_imu_pitch_deg",
    "right_imu_pitch_deg",
    "left_imu_yaw_deg",
    "right_imu_yaw_deg",
    "left_acc_x_mps2",
    "left_acc_y_mps2",
    "left_acc_z_mps2",
    "right_acc_x_mps2",
    "right_acc_y_mps2",
    "right_acc_z_mps2",
    "left_gyr_x_dps",
    "left_gyr_y_dps",
    "left_gyr_z_dps",
    "right_gyr_x_dps",
    "right_gyr_y_dps",
    "right_gyr_z_dps",
    "left_hip_velocity_dps",
    "right_hip_velocity_dps",
    "left_motor_velocity_dps",
    "right_motor_velocity_dps",
    "emg1_latest_v",
    "emg2_latest_v",
    "emg3_latest_v",
    "emg4_latest_v",
    "emg1_envelope_v",
    "emg2_envelope_v",
    "emg3_envelope_v",
    "emg4_envelope_v",
    "emg1_rail_fraction",
    "emg2_rail_fraction",
    "emg3_rail_fraction",
    "emg4_rail_fraction",
    "emg_block_sequence",
    "emg_total_samples",
    "marker_sequence",
    "marker_code",
    "marker_tick_ms",
    "usb_connected",
    "usb_streaming_active",
]


# Firmware protocol 1.2, retained for live compatibility with four-channel images.
FOUR_CHANNEL_ROBOT_LABELS = LEGACY_ROBOT_LABELS.copy()
FOUR_CHANNEL_ROBOT_LABELS.insert(8, "emg_configured_mask")
FOUR_CHANNEL_ROBOT_LABELS.insert(10, "emg_calibration_phase")
FOUR_CHANNEL_ROBOT_LABELS.insert(11, "emg_calibration_source")
_envelope_end = FOUR_CHANNEL_ROBOT_LABELS.index("emg4_envelope_v") + 1
FOUR_CHANNEL_ROBOT_LABELS[_envelope_end:_envelope_end] = [
    "emg1_bias_v",
    "emg2_bias_v",
    "emg3_bias_v",
    "emg4_bias_v",
    "emg1_rest_floor_v",
    "emg2_rest_floor_v",
    "emg3_rest_floor_v",
    "emg4_rest_floor_v",
    "emg1_mvc_envelope_v",
    "emg2_mvc_envelope_v",
    "emg3_mvc_envelope_v",
    "emg4_mvc_envelope_v",
    "emg1_activation",
    "emg2_activation",
    "emg3_activation",
    "emg4_activation",
]
del _envelope_end


# Firmware protocol 1.3, retained for compatibility with the 8-channel DAQ.
EIGHT_CHANNEL_ROBOT_LABELS = FOUR_CHANNEL_ROBOT_LABELS.copy()
for _field_suffix in (
    "latest_v",
    "envelope_v",
    "bias_v",
    "rest_floor_v",
    "mvc_envelope_v",
    "activation",
    "rail_fraction",
):
    _group_end = EIGHT_CHANNEL_ROBOT_LABELS.index(f"emg4_{_field_suffix}") + 1
    EIGHT_CHANNEL_ROBOT_LABELS[_group_end:_group_end] = [
        f"emg{channel}_{_field_suffix}" for channel in range(5, EMG_CHANNEL_COUNT + 1)
    ]
del _field_suffix, _group_end


# Firmware protocol 1.4, retained for compatibility with the single-phase DAQ.
SINGLE_PHASE_ROBOT_LABELS = EIGHT_CHANNEL_ROBOT_LABELS.copy()
_insert_at = SINGLE_PHASE_ROBOT_LABELS.index("emg_calibration_source") + 1
SINGLE_PHASE_ROBOT_LABELS[_insert_at:_insert_at] = [
    "emg_mvc_valid_mask",
    "emg_mvc_capture_mask",
    "emg_mvc_last_mask",
]
_insert_at = SINGLE_PHASE_ROBOT_LABELS.index("h10_fsm_state") + 1
SINGLE_PHASE_ROBOT_LABELS[_insert_at:_insert_at] = [
    "xm_control_mode",
    "left_commanded_torque_nm",
    "right_commanded_torque_nm",
    "left_commanded_current_est_a",
    "right_commanded_current_est_a",
]
_insert_at = SINGLE_PHASE_ROBOT_LABELS.index("h10_right_foot_contact") + 1
SINGLE_PHASE_ROBOT_LABELS[_insert_at:_insert_at] = ["h10_gait_state", "h10_gait_cycle"]
_insert_at = SINGLE_PHASE_ROBOT_LABELS.index("h10_forward_velocity_mps") + 1
SINGLE_PHASE_ROBOT_LABELS[_insert_at:_insert_at] = [
    "online_gait_phase_percent",
    "online_gait_phase_valid",
    "online_gait_phase_source",
    "online_gait_phase_confidence",
    "online_gait_stride_period_s",
    "online_gait_event_error_percent",
    "online_gait_event_count",
]
del _insert_at


# Keep this order identical to ROBOT_STREAM_CHANNELS in firmware user_app.c.
ROBOT_LABELS = SINGLE_PHASE_ROBOT_LABELS.copy()
_insert_at = ROBOT_LABELS.index("online_gait_event_count") + 1
BILATERAL_PHASE_LABELS = [
    "online_left_gait_phase_percent",
    "online_left_gait_phase_valid",
    "online_left_gait_phase_source",
    "online_left_gait_phase_confidence",
    "online_left_gait_stride_period_s",
    "online_left_gait_event_error_percent",
    "online_left_gait_event_count",
    "online_left_gait_anchor_age_s",
    "online_right_gait_phase_percent",
    "online_right_gait_phase_valid",
    "online_right_gait_phase_source",
    "online_right_gait_phase_confidence",
    "online_right_gait_stride_period_s",
    "online_right_gait_event_error_percent",
    "online_right_gait_event_count",
    "online_right_gait_anchor_age_s",
]
ROBOT_LABELS[_insert_at:_insert_at] = BILATERAL_PHASE_LABELS
del _insert_at


# damping_assist_control.c (and damping_assist_manual_buttons.c) register
# their own custom debug channel via XM_SetUsbCustomMeta(0xF0, ...) /
# XM_SendUsbDataWithId(&s_stream, sizeof(s_stream), 0xF0) -- the SAME wire
# module ID the full telemetry firmware (user_app.c, ROBOT_LABELS above)
# also uses for its own custom channel. Only one firmware is ever flashed at
# a time, so there's no runtime collision, but the two payloads are
# distinguished purely by byte length: StreamData_t is 14 floats = 56 bytes,
# far smaller than any ROBOT_LABELS schema variant (>=292 bytes).
DAMPING_DEBUG_LABELS = [
    "right_hip_angle_deg",
    "left_hip_angle_deg",
    "right_hip_vel_dps",
    "left_hip_vel_dps",
    "right_b_eff",
    "left_b_eff",
    "right_tau_raw_nm",  # controller's raw computed tau, BEFORE the
    "left_tau_raw_nm",   # g_control_ON gate / optional clamp
    "active_event_type",
    "reaction_state",
    "angle_scale",
    "trip_mean_com_velocity_mps",
    "right_tau_cmd_nm",  # what actually went to XM_SetAssistTorqueRH() --
    "left_tau_cmd_nm",   # the REAL sent torque; 0 whenever g_control_ON=0
]
DAMPING_DEBUG_PAYLOAD_SIZE = len(DAMPING_DEBUG_LABELS) * 4


class ProtocolError(ValueError):
    """Raised when a complete frame has invalid protocol content."""


@dataclass(frozen=True)
class PhaiPacket:
    seq_id: int
    module_id: int
    status: int
    payload: bytes
    encoded: bytes


@dataclass(frozen=True)
class EmgBlock:
    version: int
    sample_count: int
    block_sequence: int
    first_sample_tick_ms: int
    total_sample_count: int
    sample_period_us: int
    adc_full_scale: int
    adc_reference_mv: int
    channel_mask: int
    channel_count: int
    raw_counts: tuple[tuple[int, ...], ...]

    @property
    def volts_per_count(self) -> float:
        return (self.adc_reference_mv / 1000.0) / max(self.adc_full_scale, 1)


@dataclass
class DecoderStats:
    bytes_received: int = 0
    delimited_frames: int = 0
    valid_packets: int = 0
    cobs_errors: int = 0
    framing_errors: int = 0
    crc_errors: int = 0


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_decode(encoded: bytes) -> bytes:
    output = bytearray()
    index = 0
    while index < len(encoded):
        code = encoded[index]
        index += 1
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        end = index + code - 1
        if end > len(encoded):
            raise ProtocolError("truncated COBS frame")
        output.extend(encoded[index:end])
        index = end
        if code < 0xFF and index < len(encoded):
            output.append(0)
    return bytes(output)


def cobs_encode(raw: bytes) -> bytes:
    """Small encoder used by self-tests; firmware performs the real encoding."""
    output = bytearray([0])
    code_index = 0
    code = 1
    for byte in raw:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def parse_encoded_frame(encoded: bytes) -> PhaiPacket:
    decoded = cobs_decode(encoded)
    if len(decoded) < PHAI_HEADER_SIZE + PHAI_CRC_SIZE:
        raise ProtocolError("frame shorter than PhAI header")
    if decoded[0] != PHAI_SOF:
        raise ProtocolError("incorrect PhAI SOF")

    payload_bytes = decoded[1] * 4
    expected_size = PHAI_HEADER_SIZE + payload_bytes + PHAI_CRC_SIZE
    if decoded[1] == 0 or len(decoded) != expected_size:
        raise ProtocolError(
            f"incorrect PhAI length: got {len(decoded)}, expected {expected_size}"
        )

    payload_end = PHAI_HEADER_SIZE + payload_bytes
    received_crc = struct.unpack_from("<H", decoded, payload_end)[0]
    calculated_crc = crc16_ccitt(decoded[:payload_end])
    if received_crc != calculated_crc:
        raise ProtocolError(
            f"CRC mismatch: got 0x{received_crc:04X}, expected 0x{calculated_crc:04X}"
        )

    return PhaiPacket(
        seq_id=struct.unpack_from("<H", decoded, 2)[0],
        module_id=decoded[4],
        status=decoded[5],
        payload=decoded[PHAI_HEADER_SIZE:payload_end],
        encoded=encoded,
    )


def decode_robot_packet(packet: PhaiPacket) -> dict[str, float]:
    if packet.module_id != ROBOT_MODULE_ID:
        raise ProtocolError(f"not a robot packet: 0x{packet.module_id:02X}")
    schemas = (
        ROBOT_LABELS,
        SINGLE_PHASE_ROBOT_LABELS,
        EIGHT_CHANNEL_ROBOT_LABELS,
        FOUR_CHANNEL_ROBOT_LABELS,
        LEGACY_ROBOT_LABELS,
    )
    labels = next(
        (candidate for candidate in schemas if len(packet.payload) == len(candidate) * 4),
        None,
    )
    if labels is None:
        raise ProtocolError(
            f"robot payload has {len(packet.payload)} bytes; expected "
            + ", ".join(str(len(candidate) * 4) for candidate in schemas)
            + " bytes (bilateral v1.5, single-phase v1.4, 8-channel v1.3, "
            "four-channel, or legacy)"
        )
    values = struct.unpack(f"<{len(labels)}f", packet.payload)
    result = dict(zip(labels, values, strict=True))
    if labels is LEGACY_ROBOT_LABELS:
        result["emg_configured_mask"] = result["emg_adc_ready_mask"]
        result["emg_calibration_phase"] = 6.0
        result["emg_calibration_source"] = 0.0
    if labels is SINGLE_PHASE_ROBOT_LABELS:
        for suffix in (
            "phase_percent",
            "phase_valid",
            "phase_source",
            "phase_confidence",
            "stride_period_s",
            "event_error_percent",
            "event_count",
        ):
            result[f"online_left_gait_{suffix}"] = result[f"online_gait_{suffix}"]
    for label in ROBOT_LABELS:
        result.setdefault(label, float("nan"))
    return result


def is_damping_debug_payload(payload: bytes) -> bool:
    """True if a ROBOT_MODULE_ID (0xF0) packet's payload is
    damping_assist_control.c's StreamData_t (14 floats) rather than the full
    robot telemetry snapshot. Check this BEFORE calling decode_robot_packet
    -- that call would just raise ProtocolError on a 56-byte payload since
    no ROBOT_LABELS schema variant is that short."""
    return len(payload) == DAMPING_DEBUG_PAYLOAD_SIZE


def decode_damping_debug_packet(packet: PhaiPacket) -> dict[str, float]:
    if packet.module_id != ROBOT_MODULE_ID:
        raise ProtocolError(f"not a damping-debug packet: 0x{packet.module_id:02X}")
    if len(packet.payload) != DAMPING_DEBUG_PAYLOAD_SIZE:
        raise ProtocolError(
            f"damping-debug payload has {len(packet.payload)} bytes; "
            f"expected {DAMPING_DEBUG_PAYLOAD_SIZE}"
        )
    values = struct.unpack(f"<{len(DAMPING_DEBUG_LABELS)}f", packet.payload)
    return dict(zip(DAMPING_DEBUG_LABELS, values, strict=True))


def decode_emg_packet(packet: PhaiPacket) -> EmgBlock:
    if packet.module_id != EMG_MODULE_ID:
        raise ProtocolError(f"not an EMG packet: 0x{packet.module_id:02X}")
    if len(packet.payload) < EMG_HEADER_SIZE:
        raise ProtocolError(
            f"EMG payload has {len(packet.payload)} bytes; header needs {EMG_HEADER_SIZE}"
        )
    header = struct.unpack_from(EMG_HEADER_FORMAT, packet.payload, 0)
    magic, version, sample_count, block_sequence, first_tick, total_count, period_us, full_scale, reference_mv, channel_mask = header
    if (magic, version) == (EMG_MAGIC, EMG_PACKET_VERSION):
        channel_count = EMG_CHANNEL_COUNT
    elif (magic, version) == (LEGACY_EMG_MAGIC, LEGACY_EMG_PACKET_VERSION):
        channel_count = LEGACY_EMG_CHANNEL_COUNT
    else:
        raise ProtocolError(
            f"unsupported EMG schema: magic 0x{magic:08X}, version {version}"
        )
    expected_size = EMG_HEADER_SIZE + 2 * channel_count * EMG_SAMPLES_PER_BLOCK
    if len(packet.payload) != expected_size:
        raise ProtocolError(
            f"EMG payload has {len(packet.payload)} bytes; expected {expected_size}"
        )
    if sample_count > EMG_SAMPLES_PER_BLOCK:
        raise ProtocolError(f"invalid EMG sample count: {sample_count}")

    flat = struct.unpack_from(
        f"<{EMG_SAMPLES_PER_BLOCK * channel_count}H",
        packet.payload,
        EMG_HEADER_SIZE,
    )
    samples = tuple(
        tuple(flat[i * channel_count : (i + 1) * channel_count])
        for i in range(sample_count)
    )
    return EmgBlock(
        version=version,
        sample_count=sample_count,
        block_sequence=block_sequence,
        first_sample_tick_ms=first_tick,
        total_sample_count=total_count,
        sample_period_us=period_us,
        adc_full_scale=full_scale,
        adc_reference_mv=reference_mv,
        channel_mask=channel_mask,
        channel_count=channel_count,
        raw_counts=samples,
    )


class PhaiStreamDecoder:
    """Incremental zero-delimited COBS stream decoder."""

    def __init__(self, max_buffer_bytes: int = 1 << 20) -> None:
        self._buffer = bytearray()
        self.max_buffer_bytes = max_buffer_bytes
        self.stats = DecoderStats()

    def feed(self, data: bytes) -> list[PhaiPacket]:
        self.stats.bytes_received += len(data)
        self._buffer.extend(data)
        packets: list[PhaiPacket] = []

        while True:
            try:
                end = self._buffer.index(0)
            except ValueError:
                break
            encoded = bytes(self._buffer[:end])
            del self._buffer[: end + 1]
            if not encoded:
                continue
            self.stats.delimited_frames += 1
            try:
                packet = parse_encoded_frame(encoded)
            except ProtocolError as exc:
                message = str(exc)
                if "COBS" in message or "truncated" in message:
                    self.stats.cobs_errors += 1
                elif "CRC" in message:
                    self.stats.crc_errors += 1
                else:
                    self.stats.framing_errors += 1
                continue
            self.stats.valid_packets += 1
            packets.append(packet)

        if len(self._buffer) > self.max_buffer_bytes:
            self._buffer.clear()
            self.stats.framing_errors += 1
        return packets


def build_test_packet(payload: bytes, module_id: int, seq_id: int = 0, status: int = 0) -> bytes:
    """Create a PhAI packet for parser tests and logger dry runs."""
    units = (len(payload) + 3) // 4
    padded = payload + bytes(units * 4 - len(payload))
    raw = bytearray(struct.pack("<BBHBB", PHAI_SOF, units, seq_id & 0xFFFF, module_id, status))
    raw.extend(padded)
    raw.extend(struct.pack("<H", crc16_ccitt(raw)))
    return cobs_encode(bytes(raw)) + b"\x00"


def sequence_drop_count(previous: int | None, current: int, modulus: int) -> int:
    if previous is None:
        return 0
    delta = (current - previous) % modulus
    return max(0, delta - 1)


def rows_from_emg_block(block: EmgBlock) -> Iterable[dict[str, float | int | str]]:
    scale = block.volts_per_count
    active_mask = block.channel_mask & EMG_CHANNEL_MASK & ((1 << block.channel_count) - 1)
    for index, sample in enumerate(block.raw_counts):
        row: dict[str, float | int | str] = {
            "mcu_time_ms": block.first_sample_tick_ms + index * block.sample_period_us / 1000.0,
            "emg_block_sequence": block.block_sequence,
            "sample_index": index,
            "sample_period_us": block.sample_period_us,
            "emg_channel_mask": active_mask,
        }
        for channel in range(1, EMG_CHANNEL_COUNT + 1):
            if channel <= block.channel_count and active_mask & (1 << (channel - 1)):
                raw = sample[channel - 1]
                row[f"emg{channel}_raw_count"] = raw
                row[f"emg{channel}_v"] = raw * scale
            else:
                row[f"emg{channel}_raw_count"] = ""
                row[f"emg{channel}_v"] = ""
        yield row
