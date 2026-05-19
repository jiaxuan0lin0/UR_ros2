"""RealSense camera manager for policy observations."""

import numpy as np
import pyrealsense2 as rs

from policy_client.config import COLOR_FPS, COLOR_HEIGHT, COLOR_WIDTH


class CameraManager:
    def __init__(
        self,
        color_width: int = COLOR_WIDTH,
        color_height: int = COLOR_HEIGHT,
        color_fps: int = COLOR_FPS,
    ):
        self.ctx = rs.context()
        self.pipelines = []
        self.color_width = int(color_width)
        self.color_height = int(color_height)
        self.color_fps = int(color_fps)

    def start(self) -> None:
        devices = self.ctx.devices
        if len(devices) == 0:
            raise RuntimeError("No RealSense cameras detected")

        for dev in devices:
            serial = dev.get_info(rs.camera_info.serial_number)
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(
                rs.stream.color,
                self.color_width,
                self.color_height,
                rs.format.bgr8,
                self.color_fps,
            )
            pipeline.start(config)
            self.pipelines.append((serial, pipeline))

    def get_frames(self) -> dict[str, np.ndarray | None]:
        frames_dict = {}
        for serial, pipeline in self.pipelines:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            frames_dict[serial] = (
                None
                if not color_frame
                else np.asanyarray(color_frame.get_data())
            )
        return frames_dict

    def stop(self) -> None:
        for _, pipeline in self.pipelines:
            pipeline.stop()
        self.pipelines.clear()
