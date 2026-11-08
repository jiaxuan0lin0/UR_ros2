"""Direct RTDE and DH gripper clients for policy execution."""

from __future__ import annotations

import threading
import time

import numpy as np


class DirectDHGripper:
    """Direct DH Modbus gripper command thread over the UR tool tty."""

    MIN_FORCE = 20
    MAX_FORCE = 100
    MIN_SPEED = 1
    MAX_SPEED = 100
    MAX_COMMUNICATION_HZ = 50.0

    def __init__(
        self,
        node,
        *,
        port: str,
        baudrate: int,
        rate_hz: float,
        speed: float,
        force: float,
        raw_min: float,
        raw_max: float,
        position_deadband_raw: float,
        init_timeout_sec: float,
        read_state_hz: float,
    ):
        from controllers.dh_gripper import dh_modbus_gripper

        self.node = node
        self.port = str(port)
        self.baudrate = int(baudrate)
        command_hz = self._safe_communication_hz(rate_hz, min_hz=1.0)
        read_hz = self._safe_communication_hz(read_state_hz, min_hz=0.1)
        self.period_sec = 1.0 / command_hz
        self.speed = _clamp_int(speed, self.MIN_SPEED, self.MAX_SPEED)
        self.force = _clamp_int(force, self.MIN_FORCE, self.MAX_FORCE)
        self.raw_min = float(raw_min)
        self.raw_max = float(raw_max)
        self.position_deadband_raw = max(float(position_deadband_raw), 0.0)
        self.init_timeout_sec = max(float(init_timeout_sec), 0.0)
        self.read_state_period_sec = 1.0 / read_hz
        self._gripper = dh_modbus_gripper.dh_modbus_gripper()
        self._target_lock = threading.Lock()
        self._target_raw = None
        self._current_raw = None
        self._last_sent_raw = None
        self._last_read_time = 0.0
        self._stop_event = threading.Event()
        self._thread = None
        self._ready = False

    @classmethod
    def _safe_communication_hz(cls, value, min_hz: float) -> float:
        try:
            hz = float(value)
        except (TypeError, ValueError):
            hz = min_hz
        return min(max(hz, min_hz), cls.MAX_COMMUNICATION_HZ)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="dh_gripper_direct",
            daemon=True,
        )
        self._thread.start()

    def set_target_raw(self, raw_position: float) -> None:
        target = float(np.clip(raw_position, self.raw_min, self.raw_max))
        with self._target_lock:
            self._target_raw = target

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._gripper.close()
        except Exception:
            pass

    def current_normalized(self) -> float | None:
        with self._target_lock:
            current = self._current_raw
            target = self._target_raw
        value = current if current is not None else target
        if value is None:
            return None
        raw_range = self.raw_max - self.raw_min
        if raw_range <= 0.0:
            return None
        return float(np.clip((value - self.raw_min) / raw_range, 0.0, 1.0))

    def _connect_and_initialize(self) -> None:
        ret = self._gripper.open(self.port, self.baudrate)
        if ret < 0:
            raise RuntimeError(f"failed to open DH gripper port: {self.port}")

        self._gripper.Initialization()
        deadline = time.monotonic() + self.init_timeout_sec
        while time.monotonic() < deadline and not self._stop_event.is_set():
            init_state = self._gripper.GetInitState()
            if init_state == 1:
                self._gripper.SetTargetForce(self.force)
                self._gripper.SetTargetSpeed(self.speed)
                self._ready = True
                return
            time.sleep(0.2)
        raise RuntimeError("DH gripper initialization timed out")

    def _run(self) -> None:
        try:
            self._connect_and_initialize()
            self.node.get_logger().info(
                f"DH gripper direct tty ready on {self.port} @ {self.baudrate}."
            )
        except Exception as exc:
            self.node.get_logger().warn(f"DH gripper direct tty disabled: {exc}")
            return

        while not self._stop_event.is_set():
            with self._target_lock:
                target = self._target_raw
            if target is not None and (
                self._last_sent_raw is None
                or abs(target - self._last_sent_raw) >= self.position_deadband_raw
            ):
                try:
                    self._gripper.SetTargetPosition(int(round(target)))
                    self._last_sent_raw = target
                except Exception as exc:
                    self._ready = False
                    self.node.get_logger().warn(
                        f"DH gripper direct command failed: {exc}"
                    )
                    return

            now = time.monotonic()
            if now - self._last_read_time >= self.read_state_period_sec:
                self._last_read_time = now
                try:
                    current = self._gripper.GetCurrentPosition()
                    if current is not None:
                        with self._target_lock:
                            self._current_raw = float(
                                np.clip(current, self.raw_min, self.raw_max)
                            )
                except Exception:
                    pass
            time.sleep(self.period_sec)


class DirectRTDEJointServoClient:
    """Interpolate low-rate policy targets and stream servoJ over RTDE."""

    def __init__(
        self,
        node,
        *,
        robot_ip: str,
        servo_time: float = 1.0 / 500.0,
        control_frequency: float = 500.0,
        max_velocity: float = 1.05,
        max_acceleration: float = 1.4,
        lookahead_time: float = 0.06,
        servo_gain: float = 500.0,
        interpolation_mode: str = "pchip",
        max_joint_delta: float = 0.0,
        stop_acceleration: float = 2.0,
        gripper_raw_min: float = 0.0,
        gripper_raw_max: float = 1000.0,
        gripper_velocity: float = 100.0,
        gripper_effort: float = 100.0,
        gripper_command_deadband: float = 0.01,
        direct_gripper: bool = True,
        gripper_port: str = "/tmp/ttyUR",
        gripper_baudrate: int = 115200,
        gripper_rate_hz: float = 30.0,
        gripper_position_deadband_raw: float = 4.0,
        gripper_init_timeout_sec: float = 8.0,
        gripper_read_hz: float = 30.0,
        wait_timeout_sec: float = 5.0,
        result_timeout_sec: float = 15.0,
    ):
        self.node = node
        self.robot_ip = str(robot_ip)
        self.control_frequency = max(float(control_frequency), 1.0)
        self.servo_time = max(float(servo_time), 1.0e-3)
        expected_servo_time = 1.0 / self.control_frequency
        mismatch = abs(self.servo_time - expected_servo_time)
        if mismatch > max(1.0e-4, expected_servo_time * 0.10):
            self.node.get_logger().warn(
                "rtde_servoj_time_sec does not match "
                "rtde_servoj_control_frequency; aligning servoJ t from "
                f"{self.servo_time:.6f}s to {expected_servo_time:.6f}s."
            )
            self.servo_time = expected_servo_time
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.lookahead_time = float(lookahead_time)
        self.servo_gain = float(servo_gain)
        self.interpolation_mode = _normalize_interpolation_mode(interpolation_mode)
        self.max_joint_delta = float(max_joint_delta)
        self.stop_acceleration = float(stop_acceleration)
        self.gripper_raw_min = float(gripper_raw_min)
        self.gripper_raw_max = float(gripper_raw_max)
        self.gripper_velocity = float(gripper_velocity)
        self.gripper_effort = float(gripper_effort)
        self.gripper_command_deadband = max(float(gripper_command_deadband), 0.0)
        self.wait_timeout_sec = float(wait_timeout_sec)
        self.result_timeout_sec = float(result_timeout_sec)
        self.action_name = "direct_rtde_servoj"
        self.action_label = "direct RTDE servoJ"
        self._direct_gripper = None
        self._gripper_disabled_warned = False
        if direct_gripper:
            self._direct_gripper = DirectDHGripper(
                node,
                port=gripper_port,
                baudrate=gripper_baudrate,
                rate_hz=gripper_rate_hz,
                speed=gripper_velocity,
                force=gripper_effort,
                raw_min=gripper_raw_min,
                raw_max=gripper_raw_max,
                position_deadband_raw=gripper_position_deadband_raw,
                init_timeout_sec=gripper_init_timeout_sec,
                read_state_hz=gripper_read_hz,
            )
            self._direct_gripper.start()
        self._connect_lock = threading.Lock()
        self._execute_lock = threading.Lock()
        self._rtde_c = None
        self._rtde_r = None
        self._last_gripper_norm = None
        self._last_capture_warning_time = 0.0
        self._last_speed_limit_warning_time = 0.0

    def wait_ready(self) -> bool:
        try:
            self._ensure_connected()
            current = self._current_joints()
        except Exception as exc:
            self.node.get_logger().warn(
                f"direct RTDE servoJ not ready at {self.robot_ip}: {exc}"
            )
            return False
        return current is not None

    def send_goal(
        self,
        goal,
        *,
        wait_for_result: bool = True,
        wait_callback=None,
        wait_poll_sec: float = 0.1,
    ) -> tuple[bool, str]:
        del wait_for_result
        try:
            points = self._extract_points(goal)
        except ValueError as exc:
            return False, str(exc)
        if not points:
            return False, "empty direct RTDE servoJ goal"
        if not self.wait_ready():
            return False, f"direct RTDE servoJ not ready: {self.robot_ip}"
        if not self._execute_lock.acquire(blocking=False):
            return False, "direct RTDE servoJ is already executing a goal"

        stop_capture = threading.Event()
        capture_thread = None
        if wait_callback is not None:
            capture_thread = threading.Thread(
                target=self._capture_loop,
                args=(stop_capture, wait_callback, wait_poll_sec),
                daemon=True,
            )
            capture_thread.start()

        started = time.monotonic()
        servo_samples = 0
        try:
            servo_samples = self._stream_servoj(points)
        except Exception as exc:
            self.stop()
            return False, f"direct RTDE servoJ failed: {exc}"
        finally:
            stop_capture.set()
            if capture_thread is not None:
                capture_thread.join(timeout=1.0)
            self._execute_lock.release()

        elapsed = time.monotonic() - started
        horizon_sec = points[-1][2] if points else 0.0
        return (
            True,
            f"direct RTDE servoJ completed {len(points)} points "
            f"/{servo_samples} servo samples in {elapsed:.3f}s; "
            f"horizon={horizon_sec:.3f}s",
        )

    def stop(self) -> None:
        rtde_c = self._rtde_c
        if rtde_c is None:
            return
        try:
            rtde_c.servoStop()
        except Exception:
            pass

    def close(self) -> None:
        self.stop()
        if self._direct_gripper is not None:
            self._direct_gripper.close()
            self._direct_gripper = None
        rtde_c = self._rtde_c
        if rtde_c is not None:
            try:
                if rtde_c.isConnected():
                    rtde_c.stopScript()
            except Exception:
                pass
            try:
                disconnect = getattr(rtde_c, "disconnect", None)
                if disconnect is not None:
                    disconnect()
            except Exception:
                pass
        rtde_r = self._rtde_r
        if rtde_r is not None:
            try:
                disconnect = getattr(rtde_r, "disconnect", None)
                if disconnect is not None:
                    disconnect()
            except Exception:
                pass
        self._rtde_c = None
        self._rtde_r = None

    def current_gripper_position(self) -> float | None:
        if self._direct_gripper is None:
            return None
        return self._direct_gripper.current_normalized()

    def _ensure_connected(self) -> None:
        with self._connect_lock:
            if self._rtde_c is not None:
                try:
                    if self._rtde_c.isConnected():
                        return
                except Exception:
                    pass
                self.close()

            try:
                import rtde_control
                import rtde_receive
            except ImportError as exc:
                raise RuntimeError(
                    "ur_rtde is not importable; source the runtime that provides "
                    "rtde_control and rtde_receive"
                ) from exc

            self._rtde_c = rtde_control.RTDEControlInterface(
                self.robot_ip,
                self.control_frequency,
            )
            self._rtde_r = rtde_receive.RTDEReceiveInterface(
                self.robot_ip,
                self.control_frequency,
            )

    def _current_joints(self) -> np.ndarray | None:
        if self._rtde_r is None:
            return None
        joints = np.asarray(self._rtde_r.getActualQ(), dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            return None
        return joints

    def _extract_points(self, goal) -> list[tuple[np.ndarray, float, float]]:
        trajectory = getattr(goal, "trajectory", goal)
        points = []
        previous_time = 0.0
        for index, point in enumerate(getattr(trajectory, "points", [])):
            positions = np.asarray(getattr(point, "positions", []), dtype=np.float64)
            if positions.shape[0] < 7:
                raise ValueError(
                    "direct RTDE servoJ requires 7D joint+gripper points; "
                    f"point {index} has {positions.shape[0]} positions"
                )
            if not np.all(np.isfinite(positions[:7])):
                raise ValueError(f"point {index} contains non-finite positions")
            q = positions[:6].copy()
            gripper = float(np.clip(positions[6], 0.0, 1.0))
            point_time = _duration_to_sec(
                getattr(point, "time_from_start_sec", None)
            )
            if point_time <= previous_time:
                point_time = previous_time + self.servo_time
            points.append((q, gripper, point_time))
            previous_time = point_time
        return points

    def _stream_servoj(self, points: list[tuple[np.ndarray, float, float]]) -> int:
        if self._rtde_c is None:
            raise RuntimeError("RTDE control interface is not connected")
        running_q = self._current_joints()
        if running_q is None:
            raise RuntimeError("could not read current UR joint positions")

        sample_times, target_samples = _build_servo_path(
            points,
            start_q=running_q,
            servo_time=self.servo_time,
            interpolation_mode=self.interpolation_mode,
        )
        target_samples, limited_count, max_raw_step = _apply_joint_step_limit(
            target_samples,
            start_q=running_q,
            max_joint_delta=self.max_joint_delta,
        )
        if limited_count > 0:
            self._warn_speed_limited_trajectory(
                max_raw_step=max_raw_step,
                limited_count=limited_count,
                total_count=len(target_samples),
            )

        point_times = np.asarray([point[2] for point in points], dtype=np.float64)
        grippers = np.asarray([point[1] for point in points], dtype=np.float64)
        last_gripper_index = 0
        self._publish_gripper_target(float(grippers[0]))

        servo_samples = 0
        for sample_time, interpolated_q in zip(sample_times, target_samples):
            gripper_index = int(np.searchsorted(point_times, sample_time, side="left"))
            gripper_index = min(gripper_index, len(grippers) - 1)
            if gripper_index != last_gripper_index:
                self._publish_gripper_target(float(grippers[gripper_index]))
                last_gripper_index = gripper_index
            self._servo_joints(interpolated_q)
            servo_samples += 1
        return servo_samples

    def _warn_speed_limited_trajectory(
        self,
        *,
        max_raw_step: float,
        limited_count: int,
        total_count: int,
    ) -> None:
        now = time.monotonic()
        if now - self._last_speed_limit_warning_time < 2.0:
            return
        self._last_speed_limit_warning_time = now
        self.node.get_logger().warn(
            "servoJ trajectory speed-limited by joint-step limit: "
            f"max_raw_step_norm={max_raw_step:.6f}, "
            f"limited_samples={limited_count}/{total_count}, "
            f"max_joint_delta={self.max_joint_delta:.6f}."
        )

    def _servo_joints(self, target_q: np.ndarray) -> None:
        values = np.asarray(target_q, dtype=np.float64)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError(f"invalid servoJ target: {values}")
        t_start = self._rtde_c.initPeriod()
        self._rtde_c.servoJ(
            values.tolist(),
            self.max_velocity,
            self.max_acceleration,
            self.servo_time,
            self.lookahead_time,
            self.servo_gain,
        )
        self._rtde_c.waitPeriod(t_start)

    def _publish_gripper_target(self, normalized_position: float) -> None:
        normalized = float(np.clip(normalized_position, 0.0, 1.0))
        if (
            self._last_gripper_norm is not None
            and abs(normalized - self._last_gripper_norm)
            < self.gripper_command_deadband
        ):
            return
        self._last_gripper_norm = normalized

        raw_range = self.gripper_raw_max - self.gripper_raw_min
        raw = normalized * raw_range + self.gripper_raw_min
        if self._direct_gripper is not None:
            self._direct_gripper.set_target_raw(raw)
            return

        if not self._gripper_disabled_warned:
            self._gripper_disabled_warned = True
            self.node.get_logger().warn(
                "direct_gripper=false; gripper targets are ignored by the "
                "minimal RTDE client."
            )

    def _capture_loop(self, stop_event, wait_callback, wait_poll_sec: float) -> None:
        period = max(float(wait_poll_sec), 0.01)
        next_capture = time.monotonic() + period
        while not stop_event.is_set():
            remaining = next_capture - time.monotonic()
            if remaining > 0.0 and stop_event.wait(timeout=remaining):
                break
            try:
                wait_callback()
            except Exception as exc:
                now = time.monotonic()
                if now - self._last_capture_warning_time >= 2.0:
                    self._last_capture_warning_time = now
                    self.node.get_logger().warn(
                        f"execution history capture failed: {exc}"
                    )
            next_capture += period


def _duration_to_sec(duration) -> float:
    if duration is None:
        return 0.0
    if isinstance(duration, (int, float)):
        return max(float(duration), 0.0) if np.isfinite(duration) else 0.0
    sec = float(getattr(duration, "sec", 0.0) or 0.0)
    nanosec = float(getattr(duration, "nanosec", 0.0) or 0.0)
    value = sec + nanosec * 1.0e-9
    if not np.isfinite(value):
        return 0.0
    return max(value, 0.0)


def _build_servo_path(
    points: list[tuple[np.ndarray, float, float]],
    *,
    start_q: np.ndarray,
    servo_time: float,
    interpolation_mode: str,
) -> tuple[np.ndarray, np.ndarray]:
    point_times = np.asarray([point[2] for point in points], dtype=np.float64)
    horizon = float(point_times[-1])
    period = max(float(servo_time), 1.0e-6)
    sample_count = max(1, int(round(horizon / period)))
    sample_times = np.arange(1, sample_count + 1, dtype=np.float64) * period
    sample_times[-1] = horizon
    sample_times = np.minimum(sample_times, horizon)

    knot_times = np.concatenate(
        [np.zeros(1, dtype=np.float64), point_times],
    )
    knots = np.vstack(
        [
            np.asarray(start_q, dtype=np.float64).reshape(1, 6),
            np.asarray([point[0] for point in points], dtype=np.float64),
        ]
    )
    _validate_joint_knots(knot_times, knots)

    mode = _normalize_interpolation_mode(interpolation_mode)
    if mode == "linear":
        samples = _interpolate_linear_path(knot_times, knots, sample_times)
    else:
        tangents = _pchip_tangents(knot_times, knots)
        samples = _interpolate_cubic_hermite_path(
            knot_times,
            knots,
            tangents,
            sample_times,
        )
    return sample_times, samples


def _apply_joint_step_limit(
    samples: np.ndarray,
    *,
    start_q: np.ndarray,
    max_joint_delta: float,
) -> tuple[np.ndarray, int, float]:
    limit = float(max_joint_delta)
    if len(samples) == 0 or limit <= 0.0:
        return samples, 0, 0.0

    running_q = np.asarray(start_q, dtype=np.float64).copy()
    limited = np.empty_like(samples)
    limited_count = 0
    max_raw_step = 0.0
    for index, sample in enumerate(samples):
        delta = sample - running_q
        norm = float(np.linalg.norm(delta))
        max_raw_step = max(max_raw_step, norm)
        if norm > limit and norm > 0.0:
            sample = running_q + delta * (limit / norm)
            limited_count += 1
        limited[index] = sample
        running_q = sample
    return limited, limited_count, max_raw_step


def _interpolate_linear_path(
    knot_times: np.ndarray,
    knots: np.ndarray,
    sample_times: np.ndarray,
) -> np.ndarray:
    indices = _sample_segment_indices(knot_times, sample_times)
    local_times = sample_times - knot_times[indices]
    durations = knot_times[indices + 1] - knot_times[indices]
    alpha = np.clip(local_times / durations, 0.0, 1.0)
    start = knots[indices]
    end = knots[indices + 1]
    return start + alpha[:, None] * (end - start)


def _interpolate_cubic_hermite_path(
    knot_times: np.ndarray,
    knots: np.ndarray,
    tangents: np.ndarray,
    sample_times: np.ndarray,
) -> np.ndarray:
    indices = _sample_segment_indices(knot_times, sample_times)
    durations = knot_times[indices + 1] - knot_times[indices]
    s = np.clip((sample_times - knot_times[indices]) / durations, 0.0, 1.0)
    s2 = s * s
    s3 = s2 * s

    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2

    return (
        h00[:, None] * knots[indices]
        + h10[:, None] * durations[:, None] * tangents[indices]
        + h01[:, None] * knots[indices + 1]
        + h11[:, None] * durations[:, None] * tangents[indices + 1]
    )


def _sample_segment_indices(
    knot_times: np.ndarray,
    sample_times: np.ndarray,
) -> np.ndarray:
    indices = np.searchsorted(knot_times, sample_times, side="right") - 1
    return np.clip(indices, 0, len(knot_times) - 2)


def _pchip_tangents(knot_times: np.ndarray, knots: np.ndarray) -> np.ndarray:
    _validate_joint_knots(knot_times, knots)
    durations = np.diff(knot_times)
    slopes = np.diff(knots, axis=0) / durations[:, None]
    tangents = np.zeros_like(knots)
    if len(knots) == 2:
        tangents[0] = slopes[0]
        tangents[1] = slopes[0]
        return tangents

    h_prev = durations[:-1]
    h_next = durations[1:]
    slope_prev = slopes[:-1]
    slope_next = slopes[1:]
    same_direction = slope_prev * slope_next > 0.0
    w1 = (2.0 * h_next + h_prev)[:, None]
    w2 = (h_next + 2.0 * h_prev)[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        middle = (w1 + w2) / (w1 / slope_prev + w2 / slope_next)
    middle = np.where(np.isfinite(middle), middle, 0.0)
    tangents[1:-1] = np.where(same_direction, middle, 0.0)

    tangents[0] = _pchip_endpoint_slope(
        durations[0],
        durations[1],
        slopes[0],
        slopes[1],
    )
    tangents[-1] = _pchip_endpoint_slope(
        durations[-1],
        durations[-2],
        slopes[-1],
        slopes[-2],
    )
    return tangents


def _pchip_endpoint_slope(
    h0: float,
    h1: float,
    slope0: np.ndarray,
    slope1: np.ndarray,
) -> np.ndarray:
    tangent = ((2.0 * h0 + h1) * slope0 - h0 * slope1) / (h0 + h1)
    tangent = np.where(tangent * slope0 <= 0.0, 0.0, tangent)
    limit = 3.0 * slope0
    tangent = np.where(
        (slope0 * slope1 < 0.0) & (np.abs(tangent) > np.abs(limit)),
        limit,
        tangent,
    )
    return tangent


def _validate_joint_knots(knot_times: np.ndarray, knots: np.ndarray) -> None:
    if knot_times.ndim != 1 or len(knot_times) < 2:
        raise ValueError("servo interpolation requires at least two knot times")
    if knots.shape != (len(knot_times), 6):
        raise ValueError(
            "servo interpolation requires joint knots with shape "
            f"({len(knot_times)}, 6), got {knots.shape}"
        )
    if not np.all(np.isfinite(knot_times)) or not np.all(np.isfinite(knots)):
        raise ValueError("servo interpolation input contains non-finite values")
    if np.any(np.diff(knot_times) <= 0.0):
        raise ValueError("servo interpolation knot times must be increasing")


def _normalize_interpolation_mode(mode: str) -> str:
    normalized = str(mode).strip().lower()
    if normalized in {"linear", "lin"}:
        return "linear"
    if normalized in {"pchip", "cubic", "monotone_cubic", "monotone"}:
        return "pchip"
    raise ValueError(
        'rtde_servoj_interpolation must be "pchip" or "linear", '
        f"got {mode!r}"
    )


def _clamp_int(value, min_value: int, max_value: int) -> int:
    return max(int(min_value), min(int(round(value)), int(max_value)))
