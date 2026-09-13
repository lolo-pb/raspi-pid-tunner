"""MAVLink communication and response analysis for the bench PID tuner."""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    from pymavlink import mavutil
except ImportError:  # Allows the analysis functions to be tested without hardware deps.
    mavutil = None


AXES = ("roll", "pitch", "yaw")
AXIS_PARAMETERS = {
    "roll": {
        "rate_p": "ATC_RAT_RLL_P",
        "rate_i": "ATC_RAT_RLL_I",
        "rate_d": "ATC_RAT_RLL_D",
        "angle_p": "ATC_ANG_RLL_P",
    },
    "pitch": {
        "rate_p": "ATC_RAT_PIT_P",
        "rate_i": "ATC_RAT_PIT_I",
        "rate_d": "ATC_RAT_PIT_D",
        "angle_p": "ATC_ANG_PIT_P",
    },
    "yaw": {
        "rate_p": "ATC_RAT_YAW_P",
        "rate_i": "ATC_RAT_YAW_I",
        "rate_d": "ATC_RAT_YAW_D",
        "angle_p": "ATC_ANG_YAW_P",
    },
}
ALL_PARAMETERS = tuple(
    parameter
    for axis_parameters in AXIS_PARAMETERS.values()
    for parameter in axis_parameters.values()
)
MAX_RUN_SECONDS = 300.0


class TunerError(RuntimeError):
    """A safe, user-facing tuner failure."""


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _quaternion_to_euler_degrees(
    w: float, x: float, y: float, z: float
) -> tuple[float, float, float]:
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1 else math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _finite_or_none(value: Any) -> float | None:
    number = float(value)
    return number if math.isfinite(number) else None


def analyze_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Calculate recovery metrics from one selected-axis recording."""
    if len(samples) < 3:
        raise TunerError("At least three telemetry samples are required")

    errors = [float(sample["angle_error_deg"]) for sample in samples]
    peak_index = max(range(len(errors)), key=lambda index: abs(errors[index]))
    peak_error = errors[peak_index]
    peak_magnitude = abs(peak_error)
    if peak_magnitude < 1.0:
        raise TunerError("No disturbance of at least 1 degree was recorded")
    release_time = float(samples[peak_index]["t"])
    band = max(1.0, peak_magnitude * 0.05)

    suffix_max = [0.0] * len(errors)
    running_max = 0.0
    for index in range(len(errors) - 1, peak_index - 1, -1):
        running_max = max(running_max, abs(errors[index]))
        suffix_max[index] = running_max

    settled_index = next(
        (
            index
            for index in range(peak_index, len(errors))
            if suffix_max[index] <= band
        ),
        None,
    )
    settling_time = (
        round(float(samples[settled_index]["t"]) - release_time, 4)
        if settled_index is not None
        else None
    )

    peak_sign = 1.0 if peak_error >= 0 else -1.0
    opposite_peak = max((max(0.0, -peak_sign * value) for value in errors[peak_index:]), default=0.0)
    overshoot = (100.0 * opposite_peak / peak_magnitude) if peak_magnitude else 0.0

    final_start = float(samples[-1]["t"]) - 1.0
    final_errors = [
        error
        for sample, error in zip(samples, errors)
        if float(sample["t"]) >= final_start
    ]

    rate_errors = [
        float(sample["actual_rate_deg_s"]) - float(sample["target_rate_deg_s"])
        for sample in samples
        if sample.get("actual_rate_deg_s") is not None
        and sample.get("target_rate_deg_s") is not None
    ]
    rate_rmse = (
        math.sqrt(sum(value * value for value in rate_errors) / len(rate_errors))
        if rate_errors
        else None
    )

    crossings = 0
    prior_sign = 0
    for value in errors[peak_index:]:
        if abs(value) <= band:
            continue
        sign = 1 if value > 0 else -1
        if prior_sign and sign != prior_sign:
            crossings += 1
        prior_sign = sign

    actual_rates = [
        abs(float(sample["actual_rate_deg_s"]))
        for sample in samples
        if sample.get("actual_rate_deg_s") is not None
    ]
    accelerations = []
    for sample in samples:
        acceleration = sample.get("accel_m_s2") or {}
        if all(acceleration.get(axis) is not None for axis in ("x", "y", "z")):
            accelerations.append(
                math.sqrt(sum(float(acceleration[axis]) ** 2 for axis in ("x", "y", "z")))
            )

    return {
        "duration_s": round(float(samples[-1]["t"]) - float(samples[0]["t"]), 4),
        "sample_count": len(samples),
        "release_time_s": round(release_time, 4),
        "peak_error_deg": round(peak_error, 4),
        "settling_band_deg": round(band, 4),
        "settling_time_s": settling_time,
        "overshoot_percent": round(overshoot, 3),
        "steady_state_error_deg": round(_mean(final_errors) or 0.0, 4),
        "max_angular_rate_deg_s": round(max(actual_rates), 4) if actual_rates else None,
        "rate_tracking_rmse_deg_s": round(rate_rmse, 4) if rate_rmse is not None else None,
        "post_release_error_crossings": crossings,
        "acceleration_magnitude_rms_m_s2": (
            round(math.sqrt(sum(value * value for value in accelerations) / len(accelerations)), 4)
            if accelerations
            else None
        ),
        "acceleration_magnitude_peak_m_s2": round(max(accelerations), 4) if accelerations else None,
    }


class MavlinkTuner:
    """Owns the MAVLink link and exposes thread-safe tuner operations."""

    def __init__(self, device: str, baud: int, data_dir: str | Path) -> None:
        self.device = device
        self.baud = baud
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._connection: Any = None
        self._target_system = 0
        self._target_component = 0
        self._last_heartbeat = 0.0
        self._last_error: str | None = None
        self._mode = "UNKNOWN"
        self._armed = False
        self._vehicle = "Unknown"
        self._firmware = "Unknown"
        self._messages_seen: dict[str, float] = {}
        self._status_text: deque[str] = deque(maxlen=20)
        self._parameters: dict[str, float] = {}
        self._parameter_sequences: dict[str, int] = {}
        self._initial_snapshot: dict[str, float] | None = None
        self._previous_snapshot: dict[str, float] | None = None
        self._parameter_writes_locked = False
        self._ack_sequence = 0
        self._acks: deque[tuple[int, int, int]] = deque(maxlen=100)
        self._run: dict[str, Any] | None = None
        self._latest: dict[str, Any] = {
            "angles_deg": {axis: None for axis in AXES},
            "rates_deg_s": {axis: None for axis in AXES},
            "target_angles_deg": {axis: None for axis in AXES},
            "target_rates_deg_s": {axis: None for axis in AXES},
            "accel_m_s2": {axis: None for axis in ("x", "y", "z")},
            "vibration": {axis: None for axis in ("x", "y", "z")},
            "servo_outputs": [],
            "rc_channels": [],
            "battery_voltage_v": None,
            "battery_current_a": None,
            "battery_remaining_percent": None,
        }

    def start(self) -> None:
        if mavutil is None:
            raise TunerError("pymavlink is not installed")
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread:
            thread.join(timeout=3)
        with self._lock:
            self._close_connection_locked("Application stopped")

    def status(self) -> dict[str, Any]:
        with self._lock:
            heartbeat_age = time.monotonic() - self._last_heartbeat if self._last_heartbeat else None
            connected = self._connection is not None and heartbeat_age is not None and heartbeat_age <= 3.0
            return {
                "connected": connected,
                "device": self.device,
                "baud": self.baud,
                "vehicle": self._vehicle,
                "firmware": self._firmware,
                "system_id": self._target_system or None,
                "component_id": self._target_component or None,
                "mode": self._mode,
                "armed": self._armed,
                "heartbeat_age_s": round(heartbeat_age, 2) if heartbeat_age is not None else None,
                "last_error": self._last_error,
                "status_text": list(self._status_text),
                "parameters_ready": all(name in self._parameters for name in ALL_PARAMETERS),
                "parameter_writes_locked": self._parameter_writes_locked,
                "recording": self._run is not None,
                "recording_axis": self._run["axis"] if self._run else None,
            }

    def telemetry(self) -> dict[str, Any]:
        with self._lock:
            value = json.loads(json.dumps(self._latest))
            value.update(
                {
                    "timestamp": time.time(),
                    "connected": self.status()["connected"],
                    "mode": self._mode,
                    "armed": self._armed,
                    "recording": self._run is not None,
                }
            )
            return value

    def gains(self, axis: str) -> dict[str, Any]:
        axis = self._validate_axis(axis)
        with self._lock:
            mapping = AXIS_PARAMETERS[axis]
            values = {field: self._parameters.get(name) for field, name in mapping.items()}
            return {"axis": axis, "values": values, "ready": all(value is not None for value in values.values())}

    def apply_gains(self, axis: str, values: dict[str, float]) -> dict[str, Any]:
        axis = self._validate_axis(axis)
        required_fields = set(AXIS_PARAMETERS[axis])
        if set(values) != required_fields:
            raise TunerError(f"Gain fields must be exactly: {', '.join(sorted(required_fields))}")
        requested: dict[str, float] = {}
        for field, parameter in AXIS_PARAMETERS[axis].items():
            value = float(values[field])
            if not math.isfinite(value) or value < 0:
                raise TunerError(f"{field} must be a finite, nonnegative number")
            requested[parameter] = value
        self._ensure_parameter_write_allowed()
        self._previous_snapshot = self._snapshot_parameters()
        self._apply_parameters(requested)
        return self.gains(axis)

    def restore_parameters(self, snapshot: str) -> dict[str, Any]:
        if snapshot not in {"previous", "initial"}:
            raise TunerError("Snapshot must be 'previous' or 'initial'")
        self._ensure_parameter_write_allowed()
        with self._lock:
            values = self._previous_snapshot if snapshot == "previous" else self._initial_snapshot
            if not values:
                raise TunerError(f"No {snapshot} parameter snapshot is available")
            values = dict(values)
        self._apply_parameters(values)
        return {"restored": snapshot, "parameters": self._snapshot_parameters()}

    def set_stabilize_mode(self) -> dict[str, Any]:
        self._require_connected()
        with self._lock:
            if self._run:
                raise TunerError("Mode cannot be changed while recording")
            mapping = self._connection.mode_mapping() if self._connection else None
            if not mapping or "STABILIZE" not in mapping:
                raise TunerError("The connected vehicle does not advertise Stabilize mode")
            mode_id = mapping["STABILIZE"]
        command = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        self._send_command(
            command,
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(mode_id),
        )
        deadline = time.monotonic() + 5.0
        with self._condition:
            while self._mode != "STABILIZE" and time.monotonic() < deadline:
                self._condition.wait(deadline - time.monotonic())
            if self._mode != "STABILIZE":
                raise TunerError("ArduPilot acknowledged the mode command but did not enter Stabilize")
        return self.status()

    def arm(self) -> dict[str, Any]:
        self._require_connected()
        with self._lock:
            if self._run:
                raise TunerError("Cannot arm while a recording is active")
            if self._mode != "STABILIZE":
                raise TunerError("Select Stabilize mode before arming")
        self._arm_disarm(arm=True, force=False)
        return self.status()

    def disarm(self, force: bool = False) -> dict[str, Any]:
        self._require_connected()
        self._arm_disarm(arm=False, force=force)
        return self.status()

    def start_run(self, axis: str) -> dict[str, Any]:
        axis = self._validate_axis(axis)
        self._require_connected()
        with self._lock:
            if self._run:
                raise TunerError("A recording is already active")
            if not self._armed:
                raise TunerError("The vehicle must be armed before recording")
            if self._mode != "STABILIZE":
                raise TunerError("The vehicle must be in Stabilize mode")
            now = time.monotonic()
            for message_name in ("ATTITUDE", "ATTITUDE_TARGET"):
                if now - self._messages_seen.get(message_name, 0.0) > 1.0:
                    raise TunerError(f"Fresh {message_name} telemetry is required")
            gains = self.gains(axis)
            if not gains["ready"]:
                raise TunerError("PID parameters have not finished loading")
            run_id = f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{axis}-{uuid.uuid4().hex[:8]}"
            self._run = {
                "id": run_id,
                "axis": axis,
                "started_at": datetime.now(timezone.utc).isoformat(),
                "started_monotonic": now,
                "vehicle": self._vehicle,
                "firmware": self._firmware,
                "gains": gains["values"],
                "samples": [],
            }
            return {"id": run_id, "axis": axis, "started_at": self._run["started_at"]}

    def stop_run(self) -> dict[str, Any]:
        with self._lock:
            if not self._run:
                raise TunerError("No recording is active")
            return self._finish_run_locked("completed")

    def _receiver_loop(self) -> None:
        while not self._stop.is_set():
            if self._connection is None:
                self._connect_once()
                if self._connection is None:
                    self._stop.wait(2.0)
                    continue
            try:
                message = self._connection.recv_match(blocking=True, timeout=0.25)
                if message:
                    self._handle_message(message)
                self._check_link_and_run_timeout()
            except Exception as exc:  # Serial failures vary by platform/driver.
                with self._lock:
                    self._close_connection_locked(f"MAVLink receive failed: {exc}")

    def _connect_once(self) -> None:
        try:
            connection = mavutil.mavlink_connection(
                self.device,
                baud=self.baud,
                source_system=255,
                autoreconnect=False,
            )
            heartbeat = connection.wait_heartbeat(timeout=5)
            if heartbeat is None:
                connection.close()
                raise TunerError("No MAVLink heartbeat received")
            allowed_types = {
                mavutil.mavlink.MAV_TYPE_TRICOPTER,
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                mavutil.mavlink.MAV_TYPE_COAXIAL,
            }
            if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                connection.close()
                raise TunerError("The connected autopilot is not ArduPilot")
            if heartbeat.type not in allowed_types:
                connection.close()
                raise TunerError("The connected ArduPilot vehicle is not a supported multirotor")
            with self._lock:
                self._connection = connection
                self._target_system = heartbeat.get_srcSystem()
                self._target_component = heartbeat.get_srcComponent()
                self._last_error = None
                self._parameters.clear()
                self._parameter_sequences.clear()
                self._initial_snapshot = None
                self._previous_snapshot = None
                self._parameter_writes_locked = False
            self._handle_message(heartbeat)
            self._request_streams_and_parameters()
        except Exception as exc:
            with self._lock:
                self._last_error = f"Connection failed: {exc}"
                self._connection = None

    def _request_streams_and_parameters(self) -> None:
        message_rates = {
            "ATTITUDE": 50,
            "ATTITUDE_TARGET": 50,
            "HIGHRES_IMU": 50,
            "SERVO_OUTPUT_RAW": 20,
            "RC_CHANNELS": 10,
            "VIBRATION": 10,
            "SYS_STATUS": 2,
            "BATTERY_STATUS": 2,
            "AUTOPILOT_VERSION": 1,
        }
        for message_name, rate in message_rates.items():
            message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
            if message_id is not None:
                self._send_message_interval(message_id, rate)
        for name in ALL_PARAMETERS:
            self._connection.mav.param_request_read_send(
                self._target_system,
                self._target_component,
                name.encode("ascii"),
                -1,
            )

    def _send_message_interval(self, message_id: int, rate_hz: int) -> None:
        with self._send_lock:
            self._connection.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(round(1_000_000 / rate_hz)),
                0,
                0,
                0,
                0,
                0,
            )

    def _handle_message(self, message: Any) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return
        now = time.monotonic()
        with self._condition:
            self._messages_seen[message_type] = now
            if message_type == "HEARTBEAT":
                was_armed = self._armed
                self._last_heartbeat = now
                self._armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._mode = mavutil.mode_string_v10(message)
                vehicle_entry = mavutil.mavlink.enums["MAV_TYPE"].get(message.type)
                self._vehicle = vehicle_entry.name if vehicle_entry else "Unknown"
                if was_armed and not self._armed and self._run:
                    self._finish_run_locked("aborted", "Vehicle disarmed during recording")
            elif message_type == "AUTOPILOT_VERSION":
                version = int(message.flight_sw_version)
                self._firmware = f"{version >> 24}.{(version >> 16) & 0xFF}.{(version >> 8) & 0xFF}"
            elif message_type == "ATTITUDE":
                self._latest["angles_deg"] = {
                    "roll": _finite_or_none(math.degrees(message.roll)),
                    "pitch": _finite_or_none(math.degrees(message.pitch)),
                    "yaw": _finite_or_none(math.degrees(message.yaw)),
                }
                self._latest["rates_deg_s"] = {
                    "roll": _finite_or_none(math.degrees(message.rollspeed)),
                    "pitch": _finite_or_none(math.degrees(message.pitchspeed)),
                    "yaw": _finite_or_none(math.degrees(message.yawspeed)),
                }
                self._capture_sample_locked(now)
            elif message_type == "ATTITUDE_TARGET":
                roll, pitch, yaw = _quaternion_to_euler_degrees(*message.q)
                self._latest["target_angles_deg"] = {
                    "roll": _finite_or_none(roll),
                    "pitch": _finite_or_none(pitch),
                    "yaw": _finite_or_none(yaw),
                }
                self._latest["target_rates_deg_s"] = {
                    "roll": _finite_or_none(math.degrees(message.body_roll_rate)),
                    "pitch": _finite_or_none(math.degrees(message.body_pitch_rate)),
                    "yaw": _finite_or_none(math.degrees(message.body_yaw_rate)),
                }
            elif message_type == "HIGHRES_IMU":
                self._latest["accel_m_s2"] = {
                    "x": _finite_or_none(message.xacc),
                    "y": _finite_or_none(message.yacc),
                    "z": _finite_or_none(message.zacc),
                }
            elif message_type == "VIBRATION":
                self._latest["vibration"] = {
                    "x": _finite_or_none(message.vibration_x),
                    "y": _finite_or_none(message.vibration_y),
                    "z": _finite_or_none(message.vibration_z),
                }
            elif message_type == "SERVO_OUTPUT_RAW":
                self._latest["servo_outputs"] = [
                    getattr(message, f"servo{index}_raw", None) for index in range(1, 17)
                ]
            elif message_type == "RC_CHANNELS":
                self._latest["rc_channels"] = [
                    getattr(message, f"chan{index}_raw", None) for index in range(1, 19)
                ]
            elif message_type == "SYS_STATUS":
                self._latest["battery_voltage_v"] = (
                    message.voltage_battery / 1000.0 if message.voltage_battery != 0xFFFF else None
                )
                self._latest["battery_current_a"] = (
                    message.current_battery / 100.0 if message.current_battery != -1 else None
                )
                self._latest["battery_remaining_percent"] = (
                    message.battery_remaining if message.battery_remaining != -1 else None
                )
            elif message_type == "PARAM_VALUE":
                name = message.param_id
                if isinstance(name, bytes):
                    name = name.decode("ascii", errors="replace")
                name = name.rstrip("\x00")
                self._parameters[name] = float(message.param_value)
                self._parameter_sequences[name] = self._parameter_sequences.get(name, 0) + 1
                if self._initial_snapshot is None and all(name in self._parameters for name in ALL_PARAMETERS):
                    self._initial_snapshot = self._snapshot_parameters()
            elif message_type == "COMMAND_ACK":
                self._ack_sequence += 1
                self._acks.append((self._ack_sequence, int(message.command), int(message.result)))
            elif message_type == "STATUSTEXT":
                text = message.text.decode(errors="replace") if isinstance(message.text, bytes) else str(message.text)
                self._status_text.append(text.rstrip("\x00"))
            self._condition.notify_all()

    def _capture_sample_locked(self, now: float) -> None:
        if not self._run:
            return
        elapsed = now - self._run["started_monotonic"]
        if elapsed >= MAX_RUN_SECONDS:
            self._finish_run_locked("aborted", "Maximum recording duration reached")
            return
        axis = self._run["axis"]
        actual_angle = self._latest["angles_deg"].get(axis)
        target_angle = self._latest["target_angles_deg"].get(axis)
        if actual_angle is None or target_angle is None:
            return
        self._run["samples"].append(
            {
                "t": round(elapsed, 6),
                "target_angle_deg": round(float(target_angle), 6),
                "actual_angle_deg": round(float(actual_angle), 6),
                "angle_error_deg": round(_wrap_degrees(float(actual_angle) - float(target_angle)), 6),
                "target_rate_deg_s": self._latest["target_rates_deg_s"].get(axis),
                "actual_rate_deg_s": self._latest["rates_deg_s"].get(axis),
                "accel_m_s2": dict(self._latest["accel_m_s2"]),
            }
        )

    def _check_link_and_run_timeout(self) -> None:
        with self._lock:
            if self._connection and self._last_heartbeat and time.monotonic() - self._last_heartbeat > 3.0:
                self._close_connection_locked("MAVLink heartbeat lost")

    def _close_connection_locked(self, reason: str) -> None:
        if self._run:
            self._finish_run_locked("aborted", reason)
        connection = self._connection
        self._connection = None
        self._armed = False
        self._mode = "UNKNOWN"
        self._last_error = reason
        if connection:
            try:
                connection.close()
            except Exception:
                pass

    def _finish_run_locked(self, status: str, reason: str | None = None) -> dict[str, Any]:
        run = self._run
        if not run:
            raise TunerError("No recording is active")
        self._run = None
        run.pop("started_monotonic", None)
        run["stopped_at"] = datetime.now(timezone.utc).isoformat()
        run["status"] = status
        run["reason"] = reason
        try:
            run["metrics"] = analyze_samples(run["samples"])
        except TunerError as exc:
            run["metrics"] = None
            run["analysis_error"] = str(exc)
        path = self._run_path(run["id"])
        temporary_path = path.with_suffix(".json.tmp")
        temporary_path.write_text(json.dumps(run, indent=2), encoding="utf-8")
        temporary_path.replace(path)
        return self._run_summary(run)

    def _run_summary(self, run: dict[str, Any]) -> dict[str, Any]:
        return {
            "id": run["id"],
            "axis": run["axis"],
            "started_at": run["started_at"],
            "stopped_at": run.get("stopped_at"),
            "status": run.get("status", "unknown"),
            "reason": run.get("reason"),
            "gains": run.get("gains"),
            "metrics": run.get("metrics"),
            "sample_count": len(run.get("samples", [])),
        }

    def _run_path(self, run_id: str) -> Path:
        if not run_id or any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for character in run_id):
            raise TunerError("Invalid run identifier")
        return self.data_dir / f"{run_id}.json"

    def _arm_disarm(self, arm: bool, force: bool) -> None:
        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self._send_command(command, 1.0 if arm else 0.0, 21196.0 if force else 0.0)
        deadline = time.monotonic() + 5.0
        with self._condition:
            while self._armed == (not arm) and time.monotonic() < deadline:
                self._condition.wait(deadline - time.monotonic())
            if self._armed != arm:
                action = "arm" if arm else "disarm"
                raise TunerError(f"ArduPilot acknowledged the command but did not {action}")

    def _send_command(self, command: int, *parameters: float) -> None:
        self._require_connected()
        values = list(parameters) + [0.0] * (7 - len(parameters))
        with self._condition:
            ack_before = self._ack_sequence
            connection = self._connection
        try:
            with self._send_lock:
                connection.mav.command_long_send(
                    self._target_system,
                    self._target_component,
                    command,
                    0,
                    *values[:7],
                )
        except Exception as exc:
            raise TunerError(f"Could not send MAVLink command: {exc}") from exc
        deadline = time.monotonic() + 5.0
        with self._condition:
            while time.monotonic() < deadline:
                result = next(
                    (result for sequence, ack_command, result in self._acks if sequence > ack_before and ack_command == command),
                    None,
                )
                if result is not None:
                    accepted = {
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    }
                    if result not in accepted:
                        result_name = mavutil.mavlink.enums["MAV_RESULT"].get(result)
                        name = result_name.name if result_name else str(result)
                        raise TunerError(f"ArduPilot rejected command: {name}")
                    return
                self._condition.wait(deadline - time.monotonic())
        raise TunerError("Timed out waiting for ArduPilot command acknowledgement")

    def _ensure_parameter_write_allowed(self) -> None:
        self._require_connected()
        with self._lock:
            if self._armed:
                raise TunerError("PID parameters may only be changed while disarmed")
            if self._run:
                raise TunerError("PID parameters may not be changed while recording")
            if self._parameter_writes_locked:
                raise TunerError("Parameter writes are locked after a failed rollback; reconnect and inspect the FC")
            if not all(name in self._parameters for name in ALL_PARAMETERS):
                raise TunerError("PID parameters have not finished loading")

    def _snapshot_parameters(self) -> dict[str, float]:
        with self._lock:
            return {name: self._parameters[name] for name in ALL_PARAMETERS if name in self._parameters}

    def _apply_parameters(self, values: dict[str, float]) -> None:
        old_values = self._snapshot_parameters()
        written: list[str] = []
        try:
            for name, value in values.items():
                written.append(name)
                self._write_parameter(name, value)
        except TunerError as original_error:
            rollback_failures = []
            for name in reversed(written):
                try:
                    self._write_parameter(name, old_values[name])
                except TunerError as rollback_error:
                    rollback_failures.append(f"{name}: {rollback_error}")
            if rollback_failures:
                with self._lock:
                    self._parameter_writes_locked = True
                raise TunerError(
                    f"Parameter write failed ({original_error}); rollback also failed: {'; '.join(rollback_failures)}"
                ) from original_error
            raise TunerError(f"Parameter write failed and was rolled back: {original_error}") from original_error

    def _write_parameter(self, name: str, value: float) -> None:
        with self._condition:
            sequence_before = self._parameter_sequences.get(name, 0)
            connection = self._connection
        if connection is None:
            raise TunerError("MAVLink disconnected before the parameter write")
        try:
            with self._send_lock:
                connection.mav.param_set_send(
                    self._target_system,
                    self._target_component,
                    name.encode("ascii"),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
        except Exception as exc:
            raise TunerError(f"Could not send {name}: {exc}") from exc
        deadline = time.monotonic() + 3.0
        with self._condition:
            while time.monotonic() < deadline:
                if self._parameter_sequences.get(name, 0) > sequence_before:
                    actual = self._parameters[name]
                    tolerance = max(1e-6, abs(value) * 1e-5)
                    if not math.isclose(actual, value, rel_tol=1e-5, abs_tol=tolerance):
                        raise TunerError(f"{name} read back as {actual}, expected {value}")
                    return
                self._condition.wait(deadline - time.monotonic())
        raise TunerError(f"Timed out verifying {name}")

    def _require_connected(self) -> None:
        if not self.status()["connected"]:
            raise TunerError("No healthy MAVLink connection")

    @staticmethod
    def _validate_axis(axis: str) -> str:
        if axis not in AXES:
            raise TunerError("Axis must be roll, pitch, or yaw")
        return axis


if __name__ == "__main__":
    example_samples = [
        {
            "t": index * 0.02,
            "angle_error_deg": 15.0 * math.exp(-index * 0.04) * math.cos(index * 0.12),
            "target_rate_deg_s": 0.0,
            "actual_rate_deg_s": 0.0,
            "accel_m_s2": {"x": 0.0, "y": 0.0, "z": 9.81},
        }
        for index in range(200)
    ]
    print(json.dumps(analyze_samples(example_samples), indent=2))
