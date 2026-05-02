"""ROS 2 recorder node for segmented robot demonstration recording."""

from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from recorder.camera_manager import CameraManager
from recorder.config import (
    COLOR_FPS,
    COLOR_HEIGHT,
    COLOR_WIDTH,
    DATASET_ROOT,
    DEFAULT_RTDE_FREQUENCY,
    TELEOP_STATE_FILE,
    TELEOP_STATE_TIMEOUT_SEC,
    UR_IP,
)
from recorder.dataset_writer import DatasetWriter
from recorder.rtde_reader import RTDEReader
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
        self.declare_parameter("color_width", COLOR_WIDTH)
        self.declare_parameter("color_height", COLOR_HEIGHT)
        self.declare_parameter("color_fps", COLOR_FPS)
        self.declare_parameter("teleop_state_file", TELEOP_STATE_FILE)
        self.declare_parameter("teleop_state_timeout_sec", TELEOP_STATE_TIMEOUT_SEC)

        robot_ip = str(self.get_parameter("robot_ip").value)
        rtde_frequency = float(self.get_parameter("rtde_frequency").value)
        self.dataset_root = Path(str(self.get_parameter("dataset_root").value))
        color_width = int(self.get_parameter("color_width").value)
        color_height = int(self.get_parameter("color_height").value)
        color_fps = int(self.get_parameter("color_fps").value)
        teleop_state_file = str(self.get_parameter("teleop_state_file").value)
        teleop_state_timeout_sec = float(
            self.get_parameter("teleop_state_timeout_sec").value
        )

        self.camera_manager = CameraManager(
            color_width=color_width,
            color_height=color_height,
            color_fps=color_fps,
        )
        self.rtde_reader = RTDEReader(robot_ip=robot_ip, frequency=rtde_frequency)
        self.teleop_state_reader = TeleopStateReader(
            state_file=teleop_state_file,
            timeout_sec=teleop_state_timeout_sec,
        )

        self.state = self.IDLE
        self.frame_idx = 0
        self.dataset_writer = None
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._last_action_skip_log_time = 0.0

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

        self.get_logger().info(
            "Recorder is ready. Call /recorder/start to begin an episode."
        )

    def _connect_devices(self):
        self.get_logger().info("Connecting RTDE...")
        self.rtde_reader.connect()

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
                response.message = "Current episode is stopped. Save or discard it first."
                return response

            self.dataset_writer = DatasetWriter(dataset_root=self.dataset_root)
            self.frame_idx = 0
            self.state = self.RECORDING
            episode_dir = self.dataset_writer.episode_dir

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

            self.state = self.STOPPED
            episode_dir = self.dataset_writer.episode_dir

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

        dataset_writer.close(save=True)
        response.success = True
        response.message = f"Saved episode: {dataset_writer.episode_dir}"
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
                time.sleep(0.02)
                continue

            try:
                teleop_state = self.teleop_state_reader.read_state()
                if not self._has_fresh_action(teleop_state):
                    self._warn_skipped_action(teleop_state)
                    time.sleep(0.005)
                    continue

                frames_dict = self.camera_manager.get_frames()
                timestamp = time.time()
                robot_state = self.rtde_reader.read_state()
            except Exception as exc:
                self.get_logger().error(f"Failed to capture sample: {exc}")
                time.sleep(0.1)
                continue

            with self.state_lock:
                if self.state != self.RECORDING or self.dataset_writer is not dataset_writer:
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

            dataset_writer.enqueue_sample(frame_idx, frames_dict, record)

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

    @classmethod
    def _has_fresh_action(cls, teleop_state: dict) -> bool:
        if not teleop_state.get("available") or not teleop_state.get("fresh"):
            return False
        return (
            cls._vector(teleop_state.get("target_joint_positions"), 6) is not None
            and cls._vector(teleop_state.get("target_tcp_pose"), 6) is not None
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
        self.get_logger().warn(f"Skipping sample without fresh teleop action: {reason}")

    @classmethod
    def _build_record(
        cls,
        frame_idx: int,
        timestamp: float,
        robot_state: dict,
        teleop_state: dict,
    ):
        observation_joint_positions = cls._vector(robot_state.get("joint_positions"), 6)
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

        return {
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
