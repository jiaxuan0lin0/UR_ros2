import numpy as np

from teleop.transforms import matrix_to_rotvec, parts_to_pose_vector, pose_vector_to_parts, rotvec_to_matrix


class SafetyLimiter:
    def __init__(
        self,
        max_step_translation: float,
        max_step_rotation: float,
        max_joint_step: float,
        workspace_min=None,
        workspace_max=None,
    ):
        self.max_step_translation = float(max_step_translation)
        self.max_step_rotation = float(max_step_rotation)
        self.max_joint_step = float(max_joint_step)
        self.workspace_min = (
            np.array(workspace_min, dtype=np.float64) if workspace_min is not None else None
        )
        self.workspace_max = (
            np.array(workspace_max, dtype=np.float64) if workspace_max is not None else None
        )

    def clamp_workspace(self, tcp_pose: np.ndarray) -> np.ndarray:
        target = np.array(tcp_pose, dtype=np.float64, copy=True)
        if self.workspace_min is not None:
            target[:3] = np.maximum(target[:3], self.workspace_min)
        if self.workspace_max is not None:
            target[:3] = np.minimum(target[:3], self.workspace_max)
        return target

    def limit_tcp_step(self, target_pose, previous_pose) -> np.ndarray:
        target = self.clamp_workspace(target_pose)
        if previous_pose is None:
            return target

        prev_position, prev_rotation = pose_vector_to_parts(previous_pose)
        target_position, target_rotation = pose_vector_to_parts(target)

        delta_position = target_position - prev_position
        distance = float(np.linalg.norm(delta_position))
        if distance > self.max_step_translation > 0.0:
            delta_position *= self.max_step_translation / distance
        safe_position = prev_position + delta_position

        delta_rotation = target_rotation @ prev_rotation.T
        delta_rotvec = matrix_to_rotvec(delta_rotation)
        angle = float(np.linalg.norm(delta_rotvec))
        if angle > self.max_step_rotation > 0.0:
            delta_rotvec *= self.max_step_rotation / angle
        safe_rotation = rotvec_to_matrix(delta_rotvec) @ prev_rotation
        return parts_to_pose_vector(safe_position, safe_rotation)

    def limit_joint_step(self, target_q, previous_q) -> np.ndarray:
        target = np.array(target_q, dtype=np.float64)
        if previous_q is None:
            return target

        previous = np.array(previous_q, dtype=np.float64)
        delta = target - previous
        if self.max_joint_step > 0.0:
            delta = np.clip(delta, -self.max_joint_step, self.max_joint_step)
        return previous + delta

