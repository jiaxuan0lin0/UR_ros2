"""RealSense camera manager for synchronized frame capture."""

import numpy as np
import pyrealsense2 as rs
from recorder.config import COLOR_FPS, COLOR_HEIGHT, COLOR_WIDTH

class CameraManager:
    def __init__(self, color_width=COLOR_WIDTH, color_height=COLOR_HEIGHT, color_fps=COLOR_FPS):
        self.ctx = rs.context()
        self.pipelines = []
        self.color_width = color_width 
        self.color_height = color_height
        self.color_fps = color_fps

    def start(self):
        devices = self.ctx.devices
        if len(devices) == 0:
            raise RuntimeError("No RealSense cameras detected")
        
        for dev in devices:
            serial = dev.get_info(rs.camera_info.serial_number)

            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, 
                                 self.color_width, 
                                 self.color_height,
                                 rs.format.bgr8, 
                                 self.color_fps)
            pipeline.start(config)

            self.pipelines.append((serial, pipeline))

    def get_frames(self):
        frames_dict = {}

        for serial, pipeline in self.pipelines:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                frames_dict[serial] = None
                continue

            frames_dict[serial] = np.asanyarray(color_frame.get_data())

        return frames_dict
    
    def stop(self):
        for _, pipeline in self.pipelines:
            pipeline.stop()
        self.pipelines.clear()