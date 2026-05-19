import numpy as np

from teleop.transforms import matrix_to_rotvec, parts_to_pose_vector, pose_vector_to_parts, rotvec_to_matrix


class SafetyLimiter:
    def __init__(
        self,
        max_step_translation: float,
        max_step_rotation: float,
        max_joint_step: float,
        target_smoothing_sec: float = 0.0,
        max_target_lag_translation: float = 0.0,
        max_target_lag_rotation: float = 0.0,
        workspace_min=None,
        workspace_max=None,
    ):
        self.max_step_translation = float(max_step_translation)
        self.max_step_rotation = float(max_step_rotation)
        self.max_joint_step = float(max_joint_step)
        self.target_smoothing_sec = float(target_smoothing_sec)
        self.max_target_lag_translation = float(max_target_lag_translation)
        self.max_target_lag_rotation = float(max_target_lag_rotation)
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

    def limit_tcp_step(self, target_pose, previous_pose, cycle_dt=None) -> np.ndarray:
        target = self.clamp_workspace(target_pose)
        if previous_pose is None:
            return target

        prev_position, prev_rotation = pose_vector_to_parts(previous_pose)
        target_position, target_rotation = pose_vector_to_parts(target)

        delta_position = target_position - prev_position
        distance = float(np.linalg.norm(delta_position))
        if distance > self.max_step_translation > 0.0:
            delta_position *= self.max_step_translation / distance
        elif self.target_smoothing_sec > 0.0 and cycle_dt is not None:
            delta_position *= self._target_smoothing_fraction(cycle_dt)
        safe_position = prev_position + delta_position

        delta_rotation = target_rotation @ prev_rotation.T
        delta_rotvec = matrix_to_rotvec(delta_rotation)
        angle = float(np.linalg.norm(delta_rotvec))
        if angle > self.max_step_rotation > 0.0:
            delta_rotvec *= self.max_step_rotation / angle
        elif self.target_smoothing_sec > 0.0 and cycle_dt is not None:
            delta_rotvec *= self._target_smoothing_fraction(cycle_dt)
        safe_rotation = rotvec_to_matrix(delta_rotvec) @ prev_rotation
        return parts_to_pose_vector(safe_position, safe_rotation)

    def limit_tcp_rotation_step(self, target_pose, previous_pose, cycle_dt=None) -> np.ndarray:
        target = self.clamp_workspace(target_pose)
        if previous_pose is None:
            return target

        target_position, target_rotation = pose_vector_to_parts(target)
        _prev_position, prev_rotation = pose_vector_to_parts(previous_pose)

        delta_rotation = target_rotation @ prev_rotation.T
        delta_rotvec = matrix_to_rotvec(delta_rotation)
        angle = float(np.linalg.norm(delta_rotvec))
        if angle > self.max_step_rotation > 0.0:
            delta_rotvec *= self.max_step_rotation / angle
        elif self.target_smoothing_sec > 0.0 and cycle_dt is not None:
            delta_rotvec *= self._target_smoothing_fraction(cycle_dt)
        safe_rotation = rotvec_to_matrix(delta_rotvec) @ prev_rotation
        return parts_to_pose_vector(target_position, safe_rotation)

    def _target_smoothing_fraction(self, cycle_dt) -> float:
        smoothing = max(float(self.target_smoothing_sec), 1.0e-6)
        dt = max(0.0, float(cycle_dt))
        return min(1.0, dt / smoothing)

    def limit_tcp_lag(self, target_pose, current_pose) -> np.ndarray:
        target = self.clamp_workspace(target_pose)
        if current_pose is None:
            return target

        current_position, current_rotation = pose_vector_to_parts(current_pose)
        target_position, target_rotation = pose_vector_to_parts(target)

        delta_position = target_position - current_position
        distance = float(np.linalg.norm(delta_position))
        if distance > self.max_target_lag_translation > 0.0:
            delta_position *= self.max_target_lag_translation / distance
        safe_position = current_position + delta_position

        delta_rotation = target_rotation @ current_rotation.T
        delta_rotvec = matrix_to_rotvec(delta_rotation)
        angle = float(np.linalg.norm(delta_rotvec))
        if angle > self.max_target_lag_rotation > 0.0:
            delta_rotvec *= self.max_target_lag_rotation / angle
        safe_rotation = rotvec_to_matrix(delta_rotvec) @ current_rotation
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
