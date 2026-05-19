"""Observation sources for policy service calls."""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from policy_client.camera_manager import CameraManager
from policy_client.config import (
    COLOR_FPS,
    COLOR_HEIGHT,
    COLOR_WIDTH,
    DEFAULT_RTDE_FREQUENCY,
    UR_IP,
)
from policy_client.rtde_reader import RTDEReader
from policy_client.types import PolicyObservation


class URRealSenseRTDEObservationSource:
    """Collect RGB camera frames and 7D joint/gripper UR RTDE state."""

    def __init__(
        self,
        *,
        robot_ip: str = UR_IP,
        rtde_frequency: float = DEFAULT_RTDE_FREQUENCY,
        color_width: int = COLOR_WIDTH,
        color_height: int = COLOR_HEIGHT,
        color_fps: int = COLOR_FPS,
        primary_camera_serial: str = '',
        wrist_camera_serial: str = '',
        require_wrist_image: bool = True,
        camera_color_order: str = 'bgr',
        proprio_mode: str = 'joint_abs',
    ):
        self.robot_ip = robot_ip
        self.rtde_frequency = float(rtde_frequency)
        self.color_width = int(color_width)
        self.color_height = int(color_height)
        self.color_fps = int(color_fps)
        self.primary_camera_serial = primary_camera_serial.strip()
        self.wrist_camera_serial = wrist_camera_serial.strip()
        self.require_wrist_image = bool(require_wrist_image)
        self.camera_color_order = camera_color_order.lower().strip()
        self.proprio_mode = self._normalize_proprio_mode(proprio_mode)
        self.camera_manager = None
        self.rtde_reader = None
        self.connected = False

    def connect(self) -> None:
        if self.connected:
            return
        self._create_devices()
        self.rtde_reader.connect()
        try:
            self.camera_manager.start()
        except Exception:
            self.rtde_reader.disconnect()
            raise
        self.connected = True

    def disconnect(self) -> None:
        if not self.connected:
            return
        if self.camera_manager is not None:
            self.camera_manager.stop()
        if self.rtde_reader is not None:
            self.rtde_reader.disconnect()
        self.connected = False

    def read(self) -> PolicyObservation:
        if (
            not self.connected
            or self.camera_manager is None
            or self.rtde_reader is None
        ):
            raise RuntimeError('observation source is not connected')

        frames = self.camera_manager.get_frames()
        primary_serial, primary = self._select_primary_frame(frames)
        wrist_serial = ''
        wrist = None
        if self.require_wrist_image:
            wrist_serial, wrist = self._select_wrist_frame(
                frames,
                primary_serial,
            )

        robot_state = self.rtde_reader.read_state()
        proprio = self._build_proprio(robot_state)
        serials = (primary_serial,) if not wrist_serial else (
            primary_serial,
            wrist_serial,
        )
        return PolicyObservation(
            primary_image=self._to_rgb(primary),
            wrist_image=self._to_rgb(wrist) if wrist is not None else None,
            proprio=proprio,
            camera_serials=serials,
            robot_state=robot_state,
        )

    def _create_devices(self) -> None:
        self.camera_manager = CameraManager(
            color_width=self.color_width,
            color_height=self.color_height,
            color_fps=self.color_fps,
        )
        self.rtde_reader = RTDEReader(
            robot_ip=self.robot_ip,
            frequency=self.rtde_frequency,
        )

    def _select_primary_frame(
        self,
        frames: Dict[str, Optional[np.ndarray]],
    ) -> tuple[str, np.ndarray]:
        serials = sorted(frames)
        if not serials:
            raise RuntimeError('no camera frames are available')

        serial = self.primary_camera_serial or serials[0]
        frame = frames.get(serial)
        if frame is None:
            raise RuntimeError(f'primary camera frame unavailable: {serial}')
        return serial, frame

    def _select_wrist_frame(
        self,
        frames: Dict[str, Optional[np.ndarray]],
        primary_serial: str,
    ) -> tuple[str, np.ndarray]:
        if self.wrist_camera_serial:
            serial = self.wrist_camera_serial
        else:
            candidates = [
                serial for serial in sorted(frames) if serial != primary_serial
            ]
            if not candidates:
                raise RuntimeError(
                    'wrist camera serial not set and only one camera is '
                    'available'
                )
            serial = candidates[0]

        frame = frames.get(serial)
        if frame is None:
            raise RuntimeError(f'wrist camera frame unavailable: {serial}')
        return serial, frame

    def _to_rgb(self, image: np.ndarray) -> np.ndarray:
        if self.camera_color_order == 'rgb':
            return np.asarray(image).copy()
        if self.camera_color_order == 'bgr':
            return np.asarray(image)[..., ::-1].copy()
        raise ValueError(
            'camera_color_order must be "bgr" or "rgb", got '
            f'{self.camera_color_order}'
        )

    def _build_proprio(self, robot_state: dict) -> np.ndarray:
        arm_state = np.asarray(
            robot_state.get('joint_positions'),
            dtype=np.float64,
        )
        field_name = 'joint_positions'
        if arm_state.shape != (6,):
            raise RuntimeError(
                f'RTDE {field_name} must have shape (6,), got {arm_state}'
            )

        gripper_position = robot_state.get('gripper_position')
        if gripper_position is None:
            raise RuntimeError('RTDE gripper_position is unavailable')

        proprio = np.zeros(7, dtype=np.float64)
        proprio[:6] = arm_state
        proprio[6] = float(gripper_position)
        if not np.all(np.isfinite(proprio)):
            raise RuntimeError(f'proprio contains non-finite values: {proprio}')
        return proprio

    @staticmethod
    def _normalize_proprio_mode(value: str) -> str:
        normalized = str(value).strip().lower()
        if normalized in {'joint_abs', 'joint', 'joints', 'joint_positions'}:
            return 'joint_abs'
        raise ValueError(
            'proprio_mode must be "joint_abs", got '
            f'{value}'
        )
