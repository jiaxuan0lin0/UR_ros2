import numpy as np

from teleop.transforms import (
    parts_to_pose_vector,
    pose_vector_to_parts,
    scaled_rotation_delta,
    transform_xr_pose,
)


class PoseMapper:
    def __init__(self, scale_factor: float, rotation_scale: float):
        self.scale_factor = float(scale_factor)
        self.rotation_scale = float(rotation_scale)
        self._ref_xr_position = None
        self._ref_xr_rotation = None
        self._ref_tcp_position = None
        self._ref_tcp_rotation = None

    @property
    def active(self) -> bool:
        return self._ref_xr_position is not None

    def begin(self, xr_pose, tcp_pose):
        xr_position, xr_rotation = transform_xr_pose(xr_pose)
        tcp_position, tcp_rotation = pose_vector_to_parts(tcp_pose)
        self._ref_xr_position = xr_position.copy()
        self._ref_xr_rotation = xr_rotation.copy()
        self._ref_tcp_position = tcp_position.copy()
        self._ref_tcp_rotation = tcp_rotation.copy()

    def reset(self):
        self._ref_xr_position = None
        self._ref_xr_rotation = None
        self._ref_tcp_position = None
        self._ref_tcp_rotation = None

    def target_tcp_pose(self, xr_pose) -> np.ndarray:
        if not self.active:
            raise RuntimeError("PoseMapper.begin() must be called before target_tcp_pose().")

        xr_position, xr_rotation = transform_xr_pose(xr_pose)
        delta_position = (xr_position - self._ref_xr_position) * self.scale_factor
        delta_rotation = xr_rotation @ self._ref_xr_rotation.T
        delta_rotation = scaled_rotation_delta(delta_rotation, self.rotation_scale)

        target_position = self._ref_tcp_position + delta_position
        target_rotation = delta_rotation @ self._ref_tcp_rotation
        return parts_to_pose_vector(target_position, target_rotation)

