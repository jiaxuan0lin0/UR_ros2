"""ROS 2 recorder node for segmented robot demonstration recording."""

import math
from pathlib import Path
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from std_srvs.srv import Trigger

from recorder.camera_manager import CameraManager
from recorder.config import (
    COLOR_FPS,
    COLOR_HEIGHT,
    COLOR_WIDTH,
    DATASET_ROOT,
    DEFAULT_RTDE_FREQUENCY,
    RECORD_HZ,
    SAMPLED_REPLAY_FILE,
    TELEOP_STATE_FILE,
    TELEOP_STATE_TIMEOUT_SEC,
    UR_IP,
)
from recorder.dataset_writer import DatasetWriter
from recorder.rtde_reader import RTDEReader
from recorder.sampled_replay import (
    clear_sampled_replay_file,
    write_sampled_replay_file,
)
from recorder.teleop_state_reader import TeleopStateReader


class RecorderNode(Node):
    IDLE = "idle"
    RECORDING = "recording"
    STOPPED = "stopped"

    def __init__(self):
        super().__init__("recorder")

        self.declare_parameter("robot_ip", UR_IP)
        self.declare_parameter("rtde_frequency", DEFAULT_RTDE_FREQUENCY)
        self.declare_parameter("dataset_root", str(DATASET_ROOT))
        self.declare_parameter(
            "record_hz",
            RECORD_HZ,
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("color_width", COLOR_WIDTH)
        self.declare_parameter("color_height", COLOR_HEIGHT)
        self.declare_parameter("color_fps", COLOR_FPS)
        self.declare_parameter("teleop_state_file", TELEOP_STATE_FILE)
        self.declare_parameter("teleop_state_timeout_sec", TELEOP_STATE_TIMEOUT_SEC)
        self.declare_parameter("use_rtde_reader", False)
        self.declare_parameter("diagnostics_sec", 2.0)
        self.declare_parameter("sample_warning_sec", 0.04)

        robot_ip = str(self.get_parameter("robot_ip").value)
        rtde_frequency = float(self.get_parameter("rtde_frequency").value)
        self.dataset_root = Path(str(self.get_parameter("dataset_root").value))
        record_hz = float(self.get_parameter("record_hz").value)
        color_width = int(self.get_parameter("color_width").value)
        color_height = int(self.get_parameter("color_height").value)
        color_fps = int(self.get_parameter("color_fps").value)
        teleop_state_file = str(self.get_parameter("teleop_state_file").value)
        teleop_state_timeout_sec = float(
            self.get_parameter("teleop_state_timeout_sec").value
        )
        self.use_rtde_reader = self._as_bool(
            self.get_parameter("use_rtde_reader").value
        )
        self.diagnostics_sec = max(0.0, float(self.get_parameter("diagnostics_sec").value))
        self.sample_warning_sec = max(
            0.0,
            float(self.get_parameter("sample_warning_sec").value),
        )

        record_hz_error = self._record_hz_error(record_hz, color_fps)
        if record_hz_error is not None:
            raise ValueError(record_hz_error)

        self.color_fps = color_fps
        self.rate_lock = threading.Lock()
        self.record_hz = record_hz
        self.record_period_sec = 1.0 / record_hz
        self._next_sample_time = time.monotonic()

        self.camera_manager = CameraManager(
            color_width=color_width,
            color_height=color_height,
            color_fps=color_fps,
        )
        self.rtde_reader = (
            RTDEReader(robot_ip=robot_ip, frequency=rtde_frequency)
            if self.use_rtde_reader
            else None
        )
        self.teleop_state_reader = TeleopStateReader(
            state_file=teleop_state_file,
            timeout_sec=teleop_state_timeout_sec,
        )

        self.state = self.IDLE
        self.frame_idx = 0
        self.dataset_writer = None
        self.sampled_replay_file = Path(SAMPLED_REPLAY_FILE)
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._last_action_skip_log_time = 0.0
        self._last_diagnostics_log_time = time.monotonic()
        self._diagnostic_samples = 0
        self._diagnostic_max_total = 0.0
        self._diagnostic_max_camera = 0.0
        self._diagnostic_max_teleop = 0.0
        self._diagnostic_max_rtde = 0.0
        self._diagnostic_max_enqueue = 0.0
        self._diagnostic_max_late = 0.0
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.start_service = self.create_service(
            Trigger,
            "/recorder/start",
            self.start_recording_cb,
        )
        self.stop_service = self.create_service(
            Trigger,
            "/recorder/stop",
            self.stop_recording_cb,
        )
        self.save_service = self.create_service(
            Trigger,
            "/recorder/save",
            self.save_episode_cb,
        )
        self.discard_service = self.create_service(
            Trigger,
            "/recorder/discard",
            self.discard_episode_cb,
        )

        self._connect_devices()
        self.record_thread.start()

        if rtde_frequency > 0.0 and rtde_frequency < record_hz:
            self.get_logger().warn(
                "rtde_frequency is lower than record_hz; robot states may repeat. "
                "Use rtde_frequency >= record_hz for tighter alignment."
            )
        self.get_logger().info(
            f"Recorder is ready at {record_hz:g} Hz. "
            f"use_rtde_reader={self.use_rtde_reader}. "
            "Call /recorder/start to begin an episode."
        )

    def _connect_devices(self):
        if self.rtde_reader is not None:
            self.get_logger().info("Connecting RTDE...")
            self.rtde_reader.connect()
        else:
            self.get_logger().info(
                "Recorder RTDE reader disabled; using teleop state file for robot state."
            )

        self.get_logger().info("Starting cameras...")
        self.camera_manager.start()

    def start_recording_cb(self, request, response):
        del request

        with self.state_lock:
            if self.state == self.RECORDING:
                response.success = False
                response.message = "Already recording."
                return response

            if self.state == self.STOPPED:
                response.success = False
                response.message = (
                    "Current episode is stopped. Save or discard it first."
                )
                return response

            self.dataset_writer = DatasetWriter(dataset_root=self.dataset_root)
            self.frame_idx = 0
            self._reset_sample_clock()
            self.state = self.RECORDING
            episode_dir = self.dataset_writer.episode_dir
            clear_sampled_replay_file(self.sampled_replay_file)

        response.success = True
        response.message = f"Started recording: {episode_dir}"
        self.get_logger().info(response.message)
        return response

    def stop_recording_cb(self, request, response):
        del request

        with self.state_lock:
            if self.state != self.RECORDING:
                response.success = False
                response.message = "Recorder is not recording."
                return response

            dataset_writer = self.dataset_writer
            self.state = self.STOPPED
            episode_dir = dataset_writer.episode_dir

        if dataset_writer is not None:
            dataset_writer.wait_until_empty()
            payload = write_sampled_replay_file(
                self.sampled_replay_file,
                dataset_writer.records,
                episode_dir=dataset_writer.episode_dir,
            )
            self.get_logger().info(
                "Prepared sampled replay trajectory with "
                f"{payload['point_count']} points at {self.sampled_replay_file}."
            )

        response.success = True
        response.message = f"Stopped recording: {episode_dir}. Save or discard it."
        self.get_logger().info(response.message)
        return response

    def save_episode_cb(self, request, response):
        del request
        dataset_writer = None

        with self.state_lock:
            if self.state != self.STOPPED or self.dataset_writer is None:
                response.success = False
                response.message = "No stopped episode to save."
                return response

            dataset_writer = self.dataset_writer
            self.dataset_writer = None
            self.state = self.IDLE

        saved = dataset_writer.close(save=True)
        if saved:
            response.success = True
            response.message = f"Saved episode: {dataset_writer.episode_dir}"
        else:
            response.success = False
            response.message = (
                "No samples recorded; discarded empty episode: "
                f"{dataset_writer.episode_dir}"
            )
        self.get_logger().info(response.message)
        return response

    def discard_episode_cb(self, request, response):
        del request
        dataset_writer = None

        with self.state_lock:
            if self.state != self.STOPPED or self.dataset_writer is None:
                response.success = False
                response.message = "No stopped episode to discard."
                return response

            dataset_writer = self.dataset_writer
            self.dataset_writer = None
            self.state = self.IDLE

        dataset_writer.close(save=False)
        response.success = True
        response.message = "Discarded episode."
        self.get_logger().info(response.message)
        return response

    def _record_loop(self):
        while not self.stop_event.is_set():
            with self.state_lock:
                is_recording = self.state == self.RECORDING
                dataset_writer = self.dataset_writer

            if not is_recording or dataset_writer is None:
                self._reset_sample_clock()
                self.stop_event.wait(0.02)
                continue

            sample_late_sec = self._wait_for_next_sample()
            if sample_late_sec is None:
                continue
            sample_start = time.monotonic()

            try:
                t0 = time.monotonic()
                frames_dict = self.camera_manager.get_frames()
                camera_sec = time.monotonic() - t0
                timestamp = time.time()
                t0 = time.monotonic()
                teleop_state = self.teleop_state_reader.read_state()
                teleop_sec = time.monotonic() - t0
                if not self._has_fresh_action(teleop_state):
                    self._warn_skipped_action(teleop_state)
                    continue

                t0 = time.monotonic()
                robot_state = (
                    self.rtde_reader.read_state()
                    if self.rtde_reader is not None
                    else {}
                )
                rtde_sec = time.monotonic() - t0
            except Exception as exc:
                self.get_logger().error(f"Failed to capture sample: {exc}")
                self.stop_event.wait(0.1)
                continue

            with self.state_lock:
                if (
                    self.state != self.RECORDING
                    or self.dataset_writer is not dataset_writer
                ):
                    continue

                frame_idx = self.frame_idx

            record = self._build_record(
                frame_idx=frame_idx,
                timestamp=timestamp,
                robot_state=robot_state,
                teleop_state=teleop_state,
            )
            if record is None:
                self.get_logger().warn("Skipping malformed recorder sample.")
                continue

            with self.state_lock:
                if (
                    self.state != self.RECORDING
                    or self.dataset_writer is not dataset_writer
                    or self.frame_idx != frame_idx
                ):
                    continue
                self.frame_idx += 1

            t0 = time.monotonic()
            dataset_writer.enqueue_sample(frame_idx, frames_dict, record)
            enqueue_sec = time.monotonic() - t0
            total_sec = time.monotonic() - sample_start
            self._observe_sample_timing(
                total_sec=total_sec,
                camera_sec=camera_sec,
                teleop_sec=teleop_sec,
                rtde_sec=rtde_sec,
                enqueue_sec=enqueue_sec,
                late_sec=sample_late_sec,
                queue_size=dataset_writer.queue_size(),
            )

    @staticmethod
    def _record_hz_error(record_hz: float, color_fps: int):
        if not math.isfinite(record_hz) or record_hz <= 0.0:
            return "record_hz must be a finite number greater than 0"
        if record_hz > float(color_fps):
            return (
                f"record_hz={record_hz:g} cannot exceed color_fps={color_fps}; "
                "raise color_fps or lower record_hz so each sample has a camera frame"
            )
        return None

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _reset_sample_clock(self):
        with self.rate_lock:
            self._next_sample_time = time.monotonic()

    def _wait_for_next_sample(self):
        while not self.stop_event.is_set():
            with self.rate_lock:
                target_time = self._next_sample_time

            now = time.monotonic()
            sleep_sec = target_time - now
            if sleep_sec > 0.0:
                if self.stop_event.wait(min(sleep_sec, 0.1)):
                    return None
                continue

            with self.rate_lock:
                now = time.monotonic()
                period_sec = self.record_period_sec
                late_sec = max(0.0, now - self._next_sample_time)
                periods = int(late_sec / period_sec) + 1
                self._next_sample_time += periods * period_sec
            return late_sec

        return None

    def _observe_sample_timing(
        self,
        total_sec: float,
        camera_sec: float,
        teleop_sec: float,
        rtde_sec: float,
        enqueue_sec: float,
        late_sec: float,
        queue_size: int,
    ):
        self._diagnostic_samples += 1
        self._diagnostic_max_total = max(self._diagnostic_max_total, total_sec)
        self._diagnostic_max_camera = max(self._diagnostic_max_camera, camera_sec)
        self._diagnostic_max_teleop = max(self._diagnostic_max_teleop, teleop_sec)
        self._diagnostic_max_rtde = max(self._diagnostic_max_rtde, rtde_sec)
        self._diagnostic_max_enqueue = max(self._diagnostic_max_enqueue, enqueue_sec)
        self._diagnostic_max_late = max(self._diagnostic_max_late, late_sec)

        if self.sample_warning_sec > 0.0 and total_sec > self.sample_warning_sec:
            self.get_logger().warn(
                "Recorder slow sample: "
                f"total={total_sec * 1000.0:.1f}ms, "
                f"camera={camera_sec * 1000.0:.1f}ms, "
                f"teleop_state={teleop_sec * 1000.0:.1f}ms, "
                f"rtde={rtde_sec * 1000.0:.1f}ms, "
                f"enqueue={enqueue_sec * 1000.0:.1f}ms, "
                f"late={late_sec * 1000.0:.1f}ms, "
                f"queue={queue_size}"
            )

        if self.diagnostics_sec <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_diagnostics_log_time < self.diagnostics_sec:
            return
        self.get_logger().info(
            "Recorder timing max: "
            f"samples={self._diagnostic_samples}, "
            f"total={self._diagnostic_max_total * 1000.0:.1f}ms, "
            f"camera={self._diagnostic_max_camera * 1000.0:.1f}ms, "
            f"teleop_state={self._diagnostic_max_teleop * 1000.0:.1f}ms, "
            f"rtde={self._diagnostic_max_rtde * 1000.0:.1f}ms, "
            f"enqueue={self._diagnostic_max_enqueue * 1000.0:.1f}ms, "
            f"late={self._diagnostic_max_late * 1000.0:.1f}ms, "
            f"queue={queue_size}"
        )
        self._last_diagnostics_log_time = now
        self._diagnostic_samples = 0
        self._diagnostic_max_total = 0.0
        self._diagnostic_max_camera = 0.0
        self._diagnostic_max_teleop = 0.0
        self._diagnostic_max_rtde = 0.0
        self._diagnostic_max_enqueue = 0.0
        self._diagnostic_max_late = 0.0

    def _on_set_parameters(self, parameters):
        next_record_hz = None
        for parameter in parameters:
            if parameter.name != "record_hz":
                continue
            try:
                next_record_hz = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason="record_hz must be a positive number",
                )

            reason = self._record_hz_error(next_record_hz, self.color_fps)
            if reason is not None:
                return SetParametersResult(successful=False, reason=reason)

        if next_record_hz is not None:
            with self.rate_lock:
                self.record_hz = next_record_hz
                self.record_period_sec = 1.0 / next_record_hz
                self._next_sample_time = time.monotonic()
            self.get_logger().info(
                f"Recorder sample rate set to {next_record_hz:g} Hz."
            )

        return SetParametersResult(successful=True)

    @staticmethod
    def _vector(value, expected_len: int):
        if value is None:
            return None
        try:
            vector = [float(item) for item in value]
        except (TypeError, ValueError):
            return None
        if len(vector) != expected_len:
            return None
        return vector

    @staticmethod
    def _number(value):
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _has_fresh_action(self, teleop_state: dict) -> bool:
        if not teleop_state.get("available") or not teleop_state.get("fresh"):
            return False
        return (
            self._vector(teleop_state.get("target_joint_positions"), 6) is not None
            and self._vector(teleop_state.get("target_tcp_pose"), 6) is not None
        )

    def _warn_skipped_action(self, teleop_state: dict):
        now = time.monotonic()
        if now - self._last_action_skip_log_time < 2.0:
            return
        self._last_action_skip_log_time = now

        if not teleop_state.get("available"):
            reason = teleop_state.get("reason", "teleop_state_unavailable")
        elif not teleop_state.get("fresh"):
            age_sec = teleop_state.get("age_sec")
            if age_sec is not None:
                reason = f"stale teleop state age={age_sec:.3f}s"
            else:
                reason = "stale teleop state"
        else:
            reason = "missing target_joint_positions or target_tcp_pose"
        status = teleop_state.get("status")
        control_mode = teleop_state.get("control_mode")
        age_sec = teleop_state.get("age_sec")
        details = []
        if isinstance(status, str) and status:
            details.append(f"status={status}")
        if isinstance(control_mode, str) and control_mode:
            details.append(f"mode={control_mode}")
        if age_sec is not None:
            details.append(f"age={float(age_sec):.3f}s")
        grip_value = teleop_state.get("motion_grip_value")
        if grip_value is not None:
            details.append(f"grip={float(grip_value):.3f}")
        motion_enabled = teleop_state.get("motion_enabled")
        if motion_enabled is not None:
            details.append(f"motion_enabled={bool(motion_enabled)}")
        xr_age_sec = teleop_state.get("xr_age_sec")
        if xr_age_sec is not None:
            details.append(f"xr_age={float(xr_age_sec):.3f}s")
        suffix = f" ({', '.join(details)})" if details else ""
        self.get_logger().warn(
            f"Skipping sample without fresh teleop action: {reason}{suffix}"
        )

    @classmethod
    def _build_record(
        cls,
        frame_idx: int,
        timestamp: float,
        robot_state: dict,
        teleop_state: dict,
    ):
        observation_joint_positions = cls._vector(
            teleop_state.get("actual_joint_positions"),
            6,
        )
        if observation_joint_positions is None:
            observation_joint_positions = cls._vector(
                robot_state.get("joint_positions"),
                6,
            )

        observation_tcp_pose = cls._vector(teleop_state.get("actual_tcp_pose"), 6)
        if observation_tcp_pose is None:
            observation_tcp_pose = cls._vector(robot_state.get("tcp_pose"), 6)

        action_joint_positions = cls._vector(
            teleop_state.get("target_joint_positions"),
            6,
        )
        action_tcp_pose = cls._vector(teleop_state.get("target_tcp_pose"), 6)

        observation_gripper_position = cls._number(
            teleop_state.get("gripper_current_position")
        )
        if observation_gripper_position is None:
            observation_gripper_position = cls._number(
                robot_state.get("gripper_position")
            )

        action_gripper_position = cls._number(
            teleop_state.get("gripper_target_position")
        )
        if action_gripper_position is None:
            action_gripper_position = observation_gripper_position

        if (
            observation_joint_positions is None
            or observation_tcp_pose is None
            or observation_gripper_position is None
            or action_joint_positions is None
            or action_tcp_pose is None
            or action_gripper_position is None
        ):
            return None

        record = {
            "frame_idx": int(frame_idx),
            "timestamp": float(timestamp),
            "observation": {
                "joint_positions": observation_joint_positions,
                "tcp_pose": observation_tcp_pose,
                "gripper_position": observation_gripper_position,
            },
            "action": {
                "joint_positions": action_joint_positions,
                "tcp_pose": action_tcp_pose,
                "gripper_position": action_gripper_position,
            },
        }

        for key in ("status", "control_mode"):
            value = teleop_state.get(key)
            if isinstance(value, str) and value:
                record[key] = value

        return record

    def destroy_node(self):
        self.stop_event.set()
        if self.record_thread.is_alive():
            self.record_thread.join(timeout=2.0)

        with self.state_lock:
            dataset_writer = self.dataset_writer
            self.dataset_writer = None
            self.state = self.IDLE

        if dataset_writer is not None and not dataset_writer.closed:
            dataset_writer.close(save=False)

        self.camera_manager.stop()
        if self.rtde_reader is not None:
            self.rtde_reader.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
