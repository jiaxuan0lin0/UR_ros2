import copy
import math
import time

from controllers.ur_cartesian_delta_control import URCartesianDeltaControl
from controllers_msgs.action import FollowTcpRotvecDelta
from controllers_msgs.msg import TcpRotvecDeltaState
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor


class URTcpRotvecDeltaControl(URCartesianDeltaControl):
    """Action facade for TCP delta rotvec Servo control."""

    def __init__(self):
        super().__init__(
            node_name='ur_tcp_rotvec_delta_control',
            action_type=FollowTcpRotvecDelta,
            state_type=TcpRotvecDeltaState,
            action_name_parameter='tcp_rotvec_delta_action_name',
            default_action_name=(
                '/ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta'
            ),
            default_state_topic='/ur/tcp_rotvec_delta_control/state',
            default_target_pose_topic=(
                '/ur/tcp_rotvec_delta_control/target_pose'
            ),
            default_command_frame='base',
            mode_name='TCP rotvec delta',
        )
        self._active_tool0_to_tcp = None

    def _run_goal(self, goal_handle):
        self._active_tool0_to_tcp = None
        try:
            if (
                self.require_recent_tcp_pose
                and not self._wait_for_recent_tcp_pose()
            ):
                return (
                    'abort',
                    f'No recent TCP pose on {self.tcp_pose_topic}.',
                )

            tool0_pose = URCartesianDeltaControl._current_tool_pose_in_frame(
                self,
                self.servo_pose_frame,
            )
            tcp_pose = self._current_tcp_pose_in_frame(self.servo_pose_frame)
            if tool0_pose is None or tcp_pose is None:
                return (
                    'abort',
                    'Could not align RTDE TCP pose with MoveIt tool0 frame.',
                )
            self._active_tool0_to_tcp = self._relative_transform(
                tool0_pose,
                tcp_pose,
            )
            return super()._run_goal(goal_handle)
        finally:
            self._active_tool0_to_tcp = None

    def _current_tool_pose_in_frame(self, target_frame: str):
        return self._current_tcp_pose_in_frame(target_frame)

    def _current_tcp_pose_in_frame(self, target_frame: str):
        with self._state_lock:
            tcp_pose = copy.deepcopy(self._latest_tcp_pose)
        if tcp_pose is None:
            return None
        return self._transform_pose(tcp_pose, target_frame)

    def _apply_delta(
        self,
        reference_pose: PoseStamped,
        point,
        command_frame: str,
        target_frame: str,
    ) -> PoseStamped:
        target_pose = copy.deepcopy(reference_pose)
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = target_frame

        target_delta = self._vector_in_target_frame(
            point.translation,
            command_frame,
            target_frame,
            reference_pose,
        )
        target_pose.pose.position.x += target_delta.x
        target_pose.pose.position.y += target_delta.y
        target_pose.pose.position.z += target_delta.z

        delta_q = self._rotvec_to_quaternion_tuple(
            float(point.rotvec.x),
            float(point.rotvec.y),
            float(point.rotvec.z),
        )
        command_q = self._frame_orientation_in_target(
            command_frame,
            target_frame,
            reference_pose,
        )
        delta_in_target = self._quaternion_multiply(
            self._quaternion_multiply(command_q, delta_q),
            self._quaternion_conjugate(command_q),
        )
        current_q = self._quaternion_to_tuple(reference_pose.pose.orientation)
        target_q = self._quaternion_multiply(delta_in_target, current_q)
        target_pose.pose.orientation = self._tuple_to_quaternion(target_q)
        return target_pose

    def _stream_pose_target(
        self,
        target_pose: PoseStamped,
        duration_sec: float,
        goal_handle,
        *,
        start_pose: PoseStamped | None = None,
    ) -> bool:
        if start_pose is None:
            start_pose = self._current_tool_pose_in_frame(
                target_pose.header.frame_id
            )
        if start_pose is None:
            return super()._stream_pose_target(
                target_pose,
                duration_sec,
                goal_handle,
            )

        duration_sec = max(float(duration_sec), 0.0)
        if duration_sec <= 0.0:
            self._publish_tcp_target(target_pose)
            return goal_handle.is_cancel_requested

        period_sec = 1.0 / max(self.servo_command_hz, 1.0)
        start_time = time.monotonic()
        end_time = start_time + duration_sec

        while rclpy.ok() and time.monotonic() < end_time:
            if goal_handle.is_cancel_requested:
                return True
            alpha = (time.monotonic() - start_time) / duration_sec
            alpha = max(0.0, min(1.0, alpha))
            self._publish_tcp_target(
                self._interpolate_pose(start_pose, target_pose, alpha)
            )
            remaining = end_time - time.monotonic()
            time.sleep(min(period_sec, max(remaining, 0.0)))

        self._publish_tcp_target(target_pose)
        return goal_handle.is_cancel_requested

    def _wait_for_pose_target(self, target_pose: PoseStamped, goal_handle) -> bool:
        timeout_sec = max(self.pose_target_timeout_sec, 0.0)
        deadline = time.monotonic() + timeout_sec
        period_sec = 1.0 / max(self.servo_command_hz, 1.0)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False
            self._publish_tcp_target(target_pose)
            current_pose = self._current_tcp_pose_in_frame(
                target_pose.header.frame_id
            )
            if current_pose is not None:
                linear_error, angular_error = self._pose_error(
                    current_pose,
                    target_pose,
                )
                if (
                    linear_error <= self.pose_linear_tolerance
                    and angular_error <= self.pose_angular_tolerance
                ):
                    return True

            if timeout_sec <= 0.0 or time.monotonic() >= deadline:
                return False
            remaining = deadline - time.monotonic()
            time.sleep(min(period_sec, max(remaining, 0.0)))

        return False

    def _publish_current_tool_pose_target(self, target_frame: str):
        pose = URCartesianDeltaControl._current_tool_pose_in_frame(
            self,
            target_frame,
        )
        if pose is not None:
            self._publish_pose_target(pose)

    def _publish_tcp_target(self, tcp_target: PoseStamped):
        self._publish_pose_target(self._tcp_target_to_tool0_target(tcp_target))

    def _tcp_target_to_tool0_target(
        self,
        tcp_target: PoseStamped,
    ) -> PoseStamped:
        tool0_to_tcp = self._active_tool0_to_tcp
        if tool0_to_tcp is None:
            return copy.deepcopy(tcp_target)
        tcp_to_tool0 = self._invert_transform(tool0_to_tcp)
        return self._compose_pose_transform(tcp_target, tcp_to_tool0)

    def _transform_pose(
        self,
        pose: PoseStamped,
        target_frame: str,
    ):
        source_frame = pose.header.frame_id
        if self._same_frame(source_frame, target_frame):
            transformed = copy.deepcopy(pose)
            transformed.header.frame_id = target_frame
            transformed.header.stamp = self.get_clock().now().to_msg()
            return transformed

        transform = self._lookup_transform(target_frame, source_frame)
        if transform is None:
            return None

        rotation = self._quaternion_to_tuple(transform.transform.rotation)
        translation = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(transform.transform.translation.z),
        )
        pose_position = (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        )
        rotated = self._rotate_vector(pose_position, rotation)
        transformed = copy.deepcopy(pose)
        transformed.header.frame_id = target_frame
        transformed.header.stamp = self.get_clock().now().to_msg()
        transformed.pose.position.x = translation[0] + rotated[0]
        transformed.pose.position.y = translation[1] + rotated[1]
        transformed.pose.position.z = translation[2] + rotated[2]
        pose_q = self._quaternion_to_tuple(pose.pose.orientation)
        transformed.pose.orientation = self._tuple_to_quaternion(
            self._quaternion_multiply(rotation, pose_q)
        )
        return transformed

    def _relative_transform(
        self,
        parent_pose: PoseStamped,
        child_pose: PoseStamped,
    ):
        parent_q = self._quaternion_to_tuple(parent_pose.pose.orientation)
        child_q = self._quaternion_to_tuple(child_pose.pose.orientation)
        inverse_parent_q = self._quaternion_conjugate(parent_q)
        parent_position = (
            float(parent_pose.pose.position.x),
            float(parent_pose.pose.position.y),
            float(parent_pose.pose.position.z),
        )
        child_position = (
            float(child_pose.pose.position.x),
            float(child_pose.pose.position.y),
            float(child_pose.pose.position.z),
        )
        delta = (
            child_position[0] - parent_position[0],
            child_position[1] - parent_position[1],
            child_position[2] - parent_position[2],
        )
        relative_position = self._rotate_vector(delta, inverse_parent_q)
        relative_q = self._quaternion_multiply(inverse_parent_q, child_q)
        return relative_position, relative_q

    def _invert_transform(self, transform):
        translation, rotation = transform
        inverse_rotation = self._quaternion_conjugate(rotation)
        inverse_translation = self._rotate_vector(
            (
                -float(translation[0]),
                -float(translation[1]),
                -float(translation[2]),
            ),
            inverse_rotation,
        )
        return inverse_translation, inverse_rotation

    def _compose_pose_transform(self, pose: PoseStamped, transform):
        translation, rotation = transform
        pose_q = self._quaternion_to_tuple(pose.pose.orientation)
        rotated_translation = self._rotate_vector(translation, pose_q)
        composed = copy.deepcopy(pose)
        composed.header.stamp = self.get_clock().now().to_msg()
        composed.pose.position.x += rotated_translation[0]
        composed.pose.position.y += rotated_translation[1]
        composed.pose.position.z += rotated_translation[2]
        composed.pose.orientation = self._tuple_to_quaternion(
            self._quaternion_multiply(pose_q, rotation)
        )
        return composed

    def _interpolate_pose(
        self,
        start_pose: PoseStamped,
        target_pose: PoseStamped,
        alpha: float,
    ) -> PoseStamped:
        pose = copy.deepcopy(target_pose)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._lerp(
            start_pose.pose.position.x,
            target_pose.pose.position.x,
            alpha,
        )
        pose.pose.position.y = self._lerp(
            start_pose.pose.position.y,
            target_pose.pose.position.y,
            alpha,
        )
        pose.pose.position.z = self._lerp(
            start_pose.pose.position.z,
            target_pose.pose.position.z,
            alpha,
        )
        start_q = self._quaternion_to_tuple(start_pose.pose.orientation)
        target_q = self._quaternion_to_tuple(target_pose.pose.orientation)
        pose.pose.orientation = self._tuple_to_quaternion(
            self._slerp_quaternion_tuple(start_q, target_q, alpha)
        )
        return pose

    @staticmethod
    def _rotvec_to_quaternion_tuple(rx: float, ry: float, rz: float):
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle <= 1.0e-12:
            return URCartesianDeltaControl._identity_quaternion()
        half_angle = 0.5 * angle
        scale = math.sin(half_angle) / angle
        return URCartesianDeltaControl._normalize_quaternion_tuple((
            rx * scale,
            ry * scale,
            rz * scale,
            math.cos(half_angle),
        ))

    @staticmethod
    def _slerp_quaternion_tuple(start, target, alpha: float):
        alpha = max(0.0, min(1.0, float(alpha)))
        sx, sy, sz, sw = URCartesianDeltaControl._normalize_quaternion_tuple(
            start
        )
        tx, ty, tz, tw = URCartesianDeltaControl._normalize_quaternion_tuple(
            target
        )
        dot = sx * tx + sy * ty + sz * tz + sw * tw
        if dot < 0.0:
            tx, ty, tz, tw = -tx, -ty, -tz, -tw
            dot = -dot

        if dot > 0.9995:
            return URCartesianDeltaControl._normalize_quaternion_tuple((
                URTcpRotvecDeltaControl._lerp(sx, tx, alpha),
                URTcpRotvecDeltaControl._lerp(sy, ty, alpha),
                URTcpRotvecDeltaControl._lerp(sz, tz, alpha),
                URTcpRotvecDeltaControl._lerp(sw, tw, alpha),
            ))

        theta_0 = math.acos(max(-1.0, min(1.0, dot)))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * alpha
        sin_theta = math.sin(theta)
        scale_start = math.cos(theta) - dot * sin_theta / sin_theta_0
        scale_target = sin_theta / sin_theta_0
        return URCartesianDeltaControl._normalize_quaternion_tuple((
            scale_start * sx + scale_target * tx,
            scale_start * sy + scale_target * ty,
            scale_start * sz + scale_target * tz,
            scale_start * sw + scale_target * tw,
        ))

    @staticmethod
    def _lerp(start: float, target: float, alpha: float) -> float:
        return float(start) + (float(target) - float(start)) * float(alpha)


def main(args=None):
    rclpy.init(args=args)
    node = URTcpRotvecDeltaControl()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
