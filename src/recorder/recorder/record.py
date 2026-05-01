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
    UR_IP,
)
from recorder.dataset_writer import DatasetWriter
from recorder.rtde_reader import RTDEReader


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

        robot_ip = str(self.get_parameter("robot_ip").value)
        rtde_frequency = float(self.get_parameter("rtde_frequency").value)
        self.dataset_root = Path(str(self.get_parameter("dataset_root").value))
        color_width = int(self.get_parameter("color_width").value)
        color_height = int(self.get_parameter("color_height").value)
        color_fps = int(self.get_parameter("color_fps").value)

        self.camera_manager = CameraManager(
            color_width=color_width,
            color_height=color_height,
            color_fps=color_fps,
        )
        self.rtde_reader = RTDEReader(robot_ip=robot_ip, frequency=rtde_frequency)

        self.state = self.IDLE
        self.frame_idx = 0
        self.dataset_writer = None
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)

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
                self.frame_idx += 1

            record = {
                "frame_idx": frame_idx,
                "timestamp": timestamp,
                **robot_state,
            }
            dataset_writer.enqueue_sample(frame_idx, frames_dict, record)

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
