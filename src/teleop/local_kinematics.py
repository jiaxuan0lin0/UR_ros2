import os
import tempfile
from pathlib import Path

import numpy as np

from teleop.transforms import parts_to_pose_vector, rotvec_to_matrix


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = [float(v) for v in vector]
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


class LocalURKinematics:
    JOINT_NAMES = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )

    def __init__(
        self,
        ur_type: str = "ur7e",
        frame_name: str = "tool0",
        ur_description_path: str | None = None,
    ):
        try:
            import pinocchio as pin
            import xacro
        except ImportError as exc:
            raise RuntimeError(
                "speed_j_dls requires pinocchio and xacro. Run teleop in the Python "
                "environment that provides them and source ROS before starting."
            ) from exc

        self._pin = pin
        self.ur_type = str(ur_type)
        self.frame_name = str(frame_name)
        self.ur_description_path = self._resolve_ur_description_path(ur_description_path)
        self.urdf_path = self._make_urdf_file(xacro)
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.data = self.model.createData()
        self.base_frame_id = self._frame_id("base")
        self.frame_id = self._frame_id(self.frame_name)
        self._validate_joint_order()

    @staticmethod
    def _resolve_ur_description_path(path: str | None) -> Path:
        if path:
            resolved = Path(path)
        else:
            try:
                from ament_index_python.packages import get_package_share_directory

                resolved = Path(get_package_share_directory("ur_description"))
            except Exception:
                ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
                resolved = Path("/opt/ros") / ros_distro / "share" / "ur_description"

        if not resolved.exists():
            raise RuntimeError(f"ur_description path does not exist: {resolved}")
        return resolved

    def _make_urdf_file(self, xacro) -> Path:
        xacro_path = self.ur_description_path / "urdf" / "ur.urdf.xacro"
        if not xacro_path.exists():
            raise RuntimeError(f"UR xacro file does not exist: {xacro_path}")

        xml = xacro.process_file(
            str(xacro_path),
            mappings={
                "name": "ur",
                "ur_type": self.ur_type,
            },
        ).toxml()
        tmp = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=f".{self.ur_type}.urdf",
            prefix="ur_teleop_",
            delete=False,
        )
        with tmp:
            tmp.write(xml)
        return Path(tmp.name)

    def _frame_id(self, frame_name: str) -> int:
        frame_id = self.model.getFrameId(frame_name)
        if frame_id >= len(self.model.frames):
            available = ", ".join(frame.name for frame in self.model.frames)
            raise RuntimeError(
                f"frame '{frame_name}' is not in the URDF model. Available frames: {available}"
            )
        return frame_id

    def _validate_joint_order(self):
        model_joint_names = tuple(self.model.names[1:])
        if model_joint_names != self.JOINT_NAMES:
            raise RuntimeError(
                "Unexpected URDF joint order. "
                f"Expected {self.JOINT_NAMES}, got {model_joint_names}."
            )

    def frame_jacobian(self, joint_positions, tcp_offset=None) -> np.ndarray:
        q = np.array(joint_positions, dtype=np.float64)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError(f"expected 6 finite joint positions, got {q}")

        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        jacobian = np.array(jacobian, dtype=np.float64)

        offset = self._tcp_offset_translation(tcp_offset)
        if offset is None:
            return self._twist_to_ur_base(jacobian)

        placement = self.data.oMf[self.frame_id]
        world_to_tcp_delta = placement.rotation @ offset
        shifted = jacobian.copy()
        shifted[:3, :] = (
            jacobian[:3, :]
            - _skew(world_to_tcp_delta) @ jacobian[3:, :]
        )
        return self._twist_to_ur_base(shifted)

    def forward_pose(self, joint_positions, tcp_offset=None) -> np.ndarray:
        q = np.array(joint_positions, dtype=np.float64)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError(f"expected 6 finite joint positions, got {q}")

        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.frame_id]

        position = np.array(placement.translation, dtype=np.float64)
        rotation = np.array(placement.rotation, dtype=np.float64)
        tcp_offset_array = self._tcp_offset_array(tcp_offset)
        if tcp_offset_array is not None:
            position = position + rotation @ tcp_offset_array[:3]
            rotation = rotation @ rotvec_to_matrix(tcp_offset_array[3:6])
        position, rotation = self._pose_to_ur_base(position, rotation)
        return parts_to_pose_vector(position, rotation)

    def _twist_to_ur_base(self, jacobian: np.ndarray) -> np.ndarray:
        world_to_ur_base = self.data.oMf[self.base_frame_id].rotation.T
        transformed = np.array(jacobian, dtype=np.float64).copy()
        transformed[:3, :] = world_to_ur_base @ transformed[:3, :]
        transformed[3:, :] = world_to_ur_base @ transformed[3:, :]
        return transformed

    def _pose_to_ur_base(
        self,
        position: np.ndarray,
        rotation: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        base_placement = self.data.oMf[self.base_frame_id]
        world_to_ur_base = base_placement.rotation.T
        base_position = world_to_ur_base @ (
            np.array(position, dtype=np.float64)
            - np.array(base_placement.translation, dtype=np.float64)
        )
        base_rotation = world_to_ur_base @ np.array(rotation, dtype=np.float64)
        return base_position, base_rotation

    @staticmethod
    def _tcp_offset_translation(tcp_offset):
        offset = LocalURKinematics._tcp_offset_array(tcp_offset)
        if offset is None:
            return None
        translation = offset[:3].copy()
        if float(np.linalg.norm(translation)) <= 1.0e-12:
            return None
        return translation

    @staticmethod
    def _tcp_offset_array(tcp_offset):
        if tcp_offset is None:
            return None
        offset = np.array(tcp_offset, dtype=np.float64)
        if offset.size < 6 or not np.all(np.isfinite(offset[:6])):
            return None
        offset = offset[:6].copy()
        if float(np.linalg.norm(offset)) <= 1.0e-12:
            return None
        return offset
