import copy
import math
import threading
import time

from controllers_msgs.action import FollowCartesianDelta
from controllers_msgs.msg import CartesianDeltaState
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


GRIPPER_JOINT_NAME = 'gripper_joint'


class URCartesianDeltaControl(Node):
    """Action facade for Cartesian delta Servo control plus DH gripper."""

    def __init__(self):
        super().__init__('ur_cartesian_delta_control')
        self.callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._execution_lock = threading.Lock()

        self.declare_parameter(
            'cartesian_delta_action_name',
            '/ur/cartesian_delta_control/follow_cartesian_delta',
        )
        self.declare_parameter(
            'servo_pose_topic',
            '/servo_node/pose_target_cmds',
        )
        self.declare_parameter(
            'servo_command_type_service',
            '/servo_node/switch_command_type',
        )
        self.declare_parameter(
            'tcp_pose_topic',
            '/tcp_pose_broadcaster/pose',
        )
        self.declare_parameter('tcp_pose_publish_topic', '/ur/tcp_pose')
        self.declare_parameter(
            'gripper_joint_state_topic',
            '/gripper_joint_states',
        )
        self.declare_parameter(
            'gripper_trajectory_topic',
            '/gripper_trajectory',
        )
        self.declare_parameter(
            'state_topic',
            '/ur/cartesian_delta_control/state',
        )
        self.declare_parameter(
            'target_pose_publish_topic',
            '/ur/cartesian_delta_control/target_pose',
        )
        self.declare_parameter('command_frame', 'tool0')
        self.declare_parameter('servo_pose_frame', 'base_link')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('default_point_duration_sec', 1.0 / 30.0)
        self.declare_parameter('minimum_point_duration_sec', 0.004)
        self.declare_parameter('servo_command_hz', 125.0)
        self.declare_parameter('state_publish_hz', 30.0)
        self.declare_parameter('require_servo_subscriber', True)
        self.declare_parameter('set_servo_pose_command_type', True)
        self.declare_parameter('require_gripper_subscriber', True)
        self.declare_parameter('subscriber_timeout_sec', 2.0)
        self.declare_parameter('servo_service_timeout_sec', 2.0)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('require_recent_tcp_pose', True)
        self.declare_parameter('tcp_pose_timeout_sec', 1.0)
        self.declare_parameter('pose_linear_tolerance', 0.003)
        self.declare_parameter('pose_angular_tolerance', 0.03)
        self.declare_parameter('pose_target_timeout_sec', 5.0)
        self.declare_parameter('gripper_joint_name', GRIPPER_JOINT_NAME)
        self.declare_parameter('gripper_default_speed', 100.0)
        self.declare_parameter('gripper_default_force', 100.0)
        self.declare_parameter('gripper_position_tolerance', 5.0)
        self.declare_parameter('gripper_timeout_sec', 3.0)
        self.declare_parameter('gripper_settle_time_sec', 0.0)

        self.cartesian_delta_action_name = self.get_parameter(
            'cartesian_delta_action_name'
        ).value
        self.servo_pose_topic = self.get_parameter(
            'servo_pose_topic'
        ).value
        self.servo_command_type_service = self.get_parameter(
            'servo_command_type_service'
        ).value
        self.tcp_pose_topic = self.get_parameter('tcp_pose_topic').value
        self.tcp_pose_publish_topic = self.get_parameter(
            'tcp_pose_publish_topic'
        ).value
        self.gripper_joint_state_topic = self.get_parameter(
            'gripper_joint_state_topic'
        ).value
        self.gripper_trajectory_topic = self.get_parameter(
            'gripper_trajectory_topic'
        ).value
        self.state_topic = self.get_parameter('state_topic').value
        self.target_pose_publish_topic = self.get_parameter(
            'target_pose_publish_topic'
        ).value
        self.command_frame = self.get_parameter('command_frame').value
        self.servo_pose_frame = self.get_parameter('servo_pose_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.default_point_duration_sec = float(
            self.get_parameter('default_point_duration_sec').value
        )
        self.minimum_point_duration_sec = float(
            self.get_parameter('minimum_point_duration_sec').value
        )
        self.servo_command_hz = float(
            self.get_parameter('servo_command_hz').value
        )
        self.state_publish_hz = float(
            self.get_parameter('state_publish_hz').value
        )
        self.require_servo_subscriber = bool(
            self.get_parameter('require_servo_subscriber').value
        )
        self.set_servo_pose_command_type = bool(
            self.get_parameter('set_servo_pose_command_type').value
        )
        self.require_gripper_subscriber = bool(
            self.get_parameter('require_gripper_subscriber').value
        )
        self.subscriber_timeout_sec = float(
            self.get_parameter('subscriber_timeout_sec').value
        )
        self.servo_service_timeout_sec = float(
            self.get_parameter('servo_service_timeout_sec').value
        )
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.require_recent_tcp_pose = bool(
            self.get_parameter('require_recent_tcp_pose').value
        )
        self.tcp_pose_timeout_sec = float(
            self.get_parameter('tcp_pose_timeout_sec').value
        )
        self.pose_linear_tolerance = float(
            self.get_parameter('pose_linear_tolerance').value
        )
        self.pose_angular_tolerance = float(
            self.get_parameter('pose_angular_tolerance').value
        )
        self.pose_target_timeout_sec = float(
            self.get_parameter('pose_target_timeout_sec').value
        )
        self.gripper_joint_name = self.get_parameter(
            'gripper_joint_name'
        ).value
        self.gripper_default_speed = float(
            self.get_parameter('gripper_default_speed').value
        )
        self.gripper_default_force = float(
            self.get_parameter('gripper_default_force').value
        )
        self.gripper_position_tolerance = float(
            self.get_parameter('gripper_position_tolerance').value
        )
        self.gripper_timeout_sec = float(
            self.get_parameter('gripper_timeout_sec').value
        )
        self.gripper_settle_time_sec = float(
            self.get_parameter('gripper_settle_time_sec').value
        )

        self._latest_tcp_pose = None
        self._latest_tcp_pose_time = None
        self._latest_gripper_state = None
        self._latest_gripper_state_time = None
        self._busy = False
        self._active_point_index = 0
        self._active_point_count = 0
        self._status = 'idle'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tcp_pose_sub = self.create_subscription(
            PoseStamped,
            self.tcp_pose_topic,
            self._tcp_pose_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.gripper_joint_state_sub = self.create_subscription(
            JointState,
            self.gripper_joint_state_topic,
            self._gripper_joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.servo_pose_pub = self.create_publisher(
            PoseStamped,
            self.servo_pose_topic,
            10,
        )
        self.servo_command_type_client = self.create_client(
            ServoCommandType,
            self.servo_command_type_service,
            callback_group=self.callback_group,
        )
        self.gripper_trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.gripper_trajectory_topic,
            10,
        )
        self.tcp_pose_pub = self.create_publisher(
            PoseStamped,
            self.tcp_pose_publish_topic,
            10,
        )
        self.state_pub = self.create_publisher(
            CartesianDeltaState,
            self.state_topic,
            10,
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            self.target_pose_publish_topic,
            10,
        )
        self.action_server = ActionServer(
            self,
            FollowCartesianDelta,
            self.cartesian_delta_action_name,
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        state_period_sec = 1.0 / max(self.state_publish_hz, 1.0)
        self.state_timer = self.create_timer(
            state_period_sec,
            self._publish_state,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'UR Cartesian delta control ready: '
            f'action={self.cartesian_delta_action_name}, '
            f'servo_pose={self.servo_pose_topic}, '
            f'gripper={self.gripper_trajectory_topic}'
        )

    def _goal_callback(self, _goal_request):
        if self._execution_lock.locked():
            self.get_logger().warn('reject Cartesian delta goal: busy')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().info('accept Cartesian delta cancel request')
        return CancelResponse.ACCEPT

    def _tcp_pose_callback(self, msg: PoseStamped):
        tcp_pose = copy.deepcopy(msg)
        with self._state_lock:
            self._latest_tcp_pose = tcp_pose
            self._latest_tcp_pose_time = time.monotonic()
        self.tcp_pose_pub.publish(tcp_pose)

    def _gripper_joint_state_callback(self, msg: JointState):
        with self._state_lock:
            self._latest_gripper_state = copy.deepcopy(msg)
            self._latest_gripper_state_time = time.monotonic()

    def _publish_state(self):
        with self._state_lock:
            tcp_pose = copy.deepcopy(self._latest_tcp_pose)
            gripper_state = copy.deepcopy(self._latest_gripper_state)
            busy = self._busy
            active_point_index = self._active_point_index
            active_point_count = self._active_point_count
            status = self._status

        msg = CartesianDeltaState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if tcp_pose is not None:
            msg.tcp_pose = tcp_pose
        if gripper_state is not None:
            msg.gripper_state = gripper_state
        msg.busy = busy
        msg.active_point_index = active_point_index
        msg.active_point_count = active_point_count
        msg.status = status
        self.state_pub.publish(msg)

    def _execute_action(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            return self._finish_action(
                goal_handle,
                'abort',
                'UR Cartesian delta control is busy.',
            )

        self._set_status(True, 0, 0, 'starting')
        try:
            finish_state, message = self._run_goal(goal_handle)
        except Exception as exc:
            self._publish_current_tool_pose_target(self.servo_pose_frame)
            finish_state = 'abort'
            message = f'Cartesian delta execution failed: {exc}'
            self.get_logger().error(message)
        finally:
            self._set_status(False, 0, 0, 'idle')
            self._execution_lock.release()

        return self._finish_action(goal_handle, finish_state, message)

    def _run_goal(self, goal_handle):
        request = goal_handle.request
        points = list(request.points)
        if not points:
            return 'abort', 'Received empty Cartesian delta chunk.'

        command_frame = request.command_frame.strip() or self.command_frame
        point_duration_sec = self._point_duration_sec()

        if self.require_recent_tcp_pose and not self._wait_for_recent_tcp_pose():
            return (
                'abort',
                f'No recent TCP pose on {self.tcp_pose_topic}.',
            )

        start_pose = self._current_tool_pose_in_frame(self.servo_pose_frame)
        if start_pose is None:
            return (
                'abort',
                f'Could not transform {self.tool_frame} pose to '
                f'{self.servo_pose_frame}.',
            )
        target_frame = start_pose.header.frame_id

        if self.require_servo_subscriber:
            if not self._wait_for_subscriber(
                self.servo_pose_pub,
                self.servo_pose_topic,
            ):
                return (
                    'abort',
                    f'No Servo subscriber on {self.servo_pose_topic}.',
                )

        if self.set_servo_pose_command_type:
            ok, message = self._set_servo_pose_command_type()
            if not ok:
                return 'abort', message

        gripper_trajectory = self._build_gripper_trajectory(
            points,
            request.gripper_velocity,
            request.gripper_effort,
        )
        if gripper_trajectory.points:
            if self.require_gripper_subscriber:
                if not self._wait_for_subscriber(
                    self.gripper_trajectory_pub,
                    self.gripper_trajectory_topic,
                ):
                    return (
                        'abort',
                        'No gripper subscriber on '
                        f'{self.gripper_trajectory_topic}.',
                    )
            self.gripper_trajectory_pub.publish(gripper_trajectory)

        total_points = len(points)
        target_pose = copy.deepcopy(start_pose)
        self._set_status(True, 0, total_points, 'running')
        for index, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                self._publish_current_tool_pose_target(target_frame)
                return 'cancel', 'Cartesian delta goal canceled.'

            target_pose = self._apply_cartesian_delta(
                target_pose,
                point,
                command_frame,
                target_frame,
            )
            self._set_status(True, index, total_points, 'running')
            self._publish_feedback(
                goal_handle,
                index,
                total_points,
                f'executing point {index + 1}/{total_points}',
            )
            cancel_requested = self._stream_pose_target(
                target_pose,
                point_duration_sec,
                goal_handle,
            )
            if cancel_requested:
                self._publish_current_tool_pose_target(target_frame)
                return 'cancel', 'Cartesian delta goal canceled.'

        target_reached = self._wait_for_pose_target(target_pose, goal_handle)
        if goal_handle.is_cancel_requested:
            self._publish_current_tool_pose_target(target_frame)
            return 'cancel', 'Cartesian delta goal canceled.'
        if not target_reached:
            return (
                'abort',
                'Timed out waiting for final Cartesian pose target.',
            )

        final_gripper_target = self._final_gripper_target(points)
        if request.wait_for_gripper and final_gripper_target is not None:
            if not self._wait_for_gripper_target(
                final_gripper_target,
                request.gripper_position_tolerance,
                request.gripper_timeout,
                request.gripper_settle_time,
                goal_handle,
            ):
                return 'abort', 'Timed out waiting for final gripper target.'

        self._publish_feedback(
            goal_handle,
            total_points,
            total_points,
            'Cartesian delta chunk completed.',
        )
        return 'succeed', 'Cartesian delta chunk completed.'

    def _stream_pose_target(
        self,
        target_pose: PoseStamped,
        duration_sec: float,
        goal_handle,
    ) -> bool:
        period_sec = 1.0 / max(self.servo_command_hz, 1.0)
        end_time = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end_time:
            if goal_handle.is_cancel_requested:
                return True
            self._publish_pose_target(target_pose)
            remaining = end_time - time.monotonic()
            time.sleep(min(period_sec, max(remaining, 0.0)))

        return goal_handle.is_cancel_requested

    def _wait_for_pose_target(self, target_pose: PoseStamped, goal_handle) -> bool:
        timeout_sec = max(self.pose_target_timeout_sec, 0.0)
        deadline = time.monotonic() + timeout_sec
        period_sec = 1.0 / max(self.servo_command_hz, 1.0)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False
            self._publish_pose_target(target_pose)
            current_pose = self._current_tool_pose_in_frame(
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

    def _publish_pose_target(self, target_pose: PoseStamped):
        msg = copy.deepcopy(target_pose)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.servo_pose_pub.publish(msg)
        self.target_pose_pub.publish(msg)

    def _publish_current_tool_pose_target(self, target_frame: str):
        pose = self._current_tool_pose_in_frame(target_frame)
        if pose is not None:
            self._publish_pose_target(pose)

    def _apply_cartesian_delta(
        self,
        reference_pose: PoseStamped,
        point,
        command_frame: str,
        target_frame: str,
    ) -> PoseStamped:
        target_pose = copy.deepcopy(reference_pose)
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = target_frame

        delta_translation = Vector3()
        delta_translation.x = float(point.translation.x)
        delta_translation.y = float(point.translation.y)
        delta_translation.z = float(point.translation.z)
        target_delta = self._vector_in_target_frame(
            delta_translation,
            command_frame,
            target_frame,
            reference_pose,
        )
        target_pose.pose.position.x += target_delta.x
        target_pose.pose.position.y += target_delta.y
        target_pose.pose.position.z += target_delta.z

        delta_q = self._rpy_to_quaternion_tuple(
            float(point.rotation.x),
            float(point.rotation.y),
            float(point.rotation.z),
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

    def _current_tool_pose_in_frame(self, target_frame: str):
        resolved_frame = target_frame.strip() if target_frame else ''
        if not resolved_frame:
            resolved_frame = self.servo_pose_frame

        transform = self._lookup_transform(resolved_frame, self.tool_frame)
        if transform is None:
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = resolved_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
        return pose

    def _lookup_transform(self, target_frame: str, source_frame: str):
        timeout = RclpyDuration(seconds=max(self.tf_timeout_sec, 0.0))
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=timeout,
            )
        except TransformException:
            return None

    def _vector_in_target_frame(
        self,
        vector: Vector3,
        source_frame: str,
        target_frame: str,
        reference_pose: PoseStamped,
    ) -> Vector3:
        if self._same_frame(source_frame, target_frame):
            return copy.deepcopy(vector)

        if self._same_frame(source_frame, self.tool_frame):
            rotation = self._quaternion_to_tuple(reference_pose.pose.orientation)
        else:
            rotation = self._lookup_frame_orientation(source_frame, target_frame)

        x, y, z = self._rotate_vector(
            (float(vector.x), float(vector.y), float(vector.z)),
            rotation,
        )
        transformed = Vector3()
        transformed.x = x
        transformed.y = y
        transformed.z = z
        return transformed

    def _frame_orientation_in_target(
        self,
        source_frame: str,
        target_frame: str,
        reference_pose: PoseStamped,
    ):
        if self._same_frame(source_frame, target_frame):
            return self._identity_quaternion()
        if self._same_frame(source_frame, self.tool_frame):
            return self._quaternion_to_tuple(reference_pose.pose.orientation)
        return self._lookup_frame_orientation(source_frame, target_frame)

    def _lookup_frame_orientation(self, source_frame: str, target_frame: str):
        timeout = RclpyDuration(seconds=max(self.tf_timeout_sec, 0.0))
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=timeout,
            )
        except TransformException as exc:
            raise RuntimeError(
                'Could not transform Cartesian delta from '
                f'{source_frame} to {target_frame}: {exc}'
            ) from exc

        return self._quaternion_to_tuple(transform.transform.rotation)

    def _pose_error(
        self,
        current_pose: PoseStamped,
        target_pose: PoseStamped,
    ):
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        dz = target_pose.pose.position.z - current_pose.pose.position.z
        linear_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        current_q = self._quaternion_to_tuple(current_pose.pose.orientation)
        target_q = self._quaternion_to_tuple(target_pose.pose.orientation)
        error_q = self._quaternion_multiply(
            target_q,
            self._quaternion_conjugate(current_q),
        )
        error_q = self._normalize_quaternion_tuple(error_q)
        angular_error = 2.0 * math.atan2(
            math.sqrt(
                error_q[0] * error_q[0]
                + error_q[1] * error_q[1]
                + error_q[2] * error_q[2]
            ),
            abs(error_q[3]),
        )
        return linear_error, angular_error

    @staticmethod
    def _same_frame(left: str, right: str) -> bool:
        return URCartesianDeltaControl._normalize_frame(
            left
        ) == URCartesianDeltaControl._normalize_frame(right)

    @staticmethod
    def _normalize_frame(frame: str) -> str:
        return (frame or '').strip().lstrip('/')

    @staticmethod
    def _identity_quaternion():
        return 0.0, 0.0, 0.0, 1.0

    @staticmethod
    def _rpy_to_quaternion_tuple(roll: float, pitch: float, yaw: float):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)

        return URCartesianDeltaControl._normalize_quaternion_tuple((
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ))

    @staticmethod
    def _quaternion_to_tuple(quaternion):
        return URCartesianDeltaControl._normalize_quaternion_tuple((
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
        ))

    @staticmethod
    def _tuple_to_quaternion(values) -> Quaternion:
        x, y, z, w = URCartesianDeltaControl._normalize_quaternion_tuple(values)
        quaternion = Quaternion()
        quaternion.x = x
        quaternion.y = y
        quaternion.z = z
        quaternion.w = w
        return quaternion

    @staticmethod
    def _quaternion_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def _quaternion_conjugate(quaternion):
        x, y, z, w = quaternion
        return -x, -y, -z, w

    @staticmethod
    def _normalize_quaternion_tuple(quaternion):
        x, y, z, w = quaternion
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 0.0:
            return URCartesianDeltaControl._identity_quaternion()
        return x / norm, y / norm, z / norm, w / norm

    @staticmethod
    def _rotate_vector(vector, quaternion):
        vector_q = (float(vector[0]), float(vector[1]), float(vector[2]), 0.0)
        rotated = URCartesianDeltaControl._quaternion_multiply(
            URCartesianDeltaControl._quaternion_multiply(quaternion, vector_q),
            URCartesianDeltaControl._quaternion_conjugate(quaternion),
        )
        return rotated[0], rotated[1], rotated[2]

    def _build_gripper_trajectory(
        self,
        points,
        gripper_velocity: float,
        gripper_effort: float,
    ) -> JointTrajectory:
        velocity = (
            float(gripper_velocity)
            if gripper_velocity > 0.0
            else self.gripper_default_speed
        )
        effort = (
            float(gripper_effort)
            if gripper_effort > 0.0
            else self.gripper_default_force
        )

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [self.gripper_joint_name]

        for point in points:
            if not math.isfinite(float(point.gripper)):
                continue

            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = [float(point.gripper)]
            trajectory_point.velocities = [velocity]
            trajectory_point.effort = [effort]
            trajectory.points.append(trajectory_point)

        return trajectory

    def _wait_for_gripper_target(
        self,
        target: float,
        tolerance: float,
        timeout_sec: float,
        settle_time_sec: float,
        goal_handle,
    ) -> bool:
        tolerance = (
            float(tolerance)
            if tolerance > 0.0
            else self.gripper_position_tolerance
        )
        timeout_sec = (
            float(timeout_sec)
            if timeout_sec > 0.0
            else self.gripper_timeout_sec
        )
        settle_time_sec = (
            float(settle_time_sec)
            if settle_time_sec > 0.0
            else self.gripper_settle_time_sec
        )

        if settle_time_sec > 0.0:
            end_settle = time.monotonic() + settle_time_sec
            while rclpy.ok() and time.monotonic() < end_settle:
                if goal_handle.is_cancel_requested:
                    return False
                time.sleep(0.02)

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            current_position = self._current_gripper_position()
            if current_position is not None:
                if abs(current_position - target) <= tolerance:
                    return True
            time.sleep(0.02)
        return False

    def _current_gripper_position(self):
        with self._state_lock:
            state = copy.deepcopy(self._latest_gripper_state)
        if state is None or not state.position:
            return None

        try:
            index = state.name.index(self.gripper_joint_name)
        except ValueError:
            index = 0
        if index >= len(state.position):
            return None
        return float(state.position[index])

    def _final_gripper_target(self, points):
        for point in reversed(points):
            gripper = float(point.gripper)
            if math.isfinite(gripper):
                return gripper
        return None

    def _point_duration_sec(self) -> float:
        return max(
            self.default_point_duration_sec,
            self.minimum_point_duration_sec,
        )

    def _wait_for_recent_tcp_pose(self) -> bool:
        deadline = time.monotonic() + max(self.tcp_pose_timeout_sec, 0.0)
        while rclpy.ok() and time.monotonic() <= deadline:
            with self._state_lock:
                latest_time = self._latest_tcp_pose_time
            if latest_time is not None:
                if time.monotonic() - latest_time <= self.tcp_pose_timeout_sec:
                    return True
            time.sleep(0.02)
        return False

    def _wait_for_subscriber(self, publisher, topic_name: str) -> bool:
        deadline = time.monotonic() + max(self.subscriber_timeout_sec, 0.0)
        while rclpy.ok() and time.monotonic() <= deadline:
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.02)

        self.get_logger().warn(f'timed out waiting for subscriber: {topic_name}')
        return False

    def _set_servo_pose_command_type(self):
        if not self.servo_command_type_client.wait_for_service(
            timeout_sec=self.servo_service_timeout_sec
        ):
            return (
                False,
                'Servo command type service not ready: '
                f'{self.servo_command_type_service}.',
            )

        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.POSE
        future = self.servo_command_type_client.call_async(request)
        if not self._wait_for_future_done(
            future,
            self.servo_service_timeout_sec,
        ):
            return (
                False,
                'Timed out switching Servo command type to POSE.',
            )

        try:
            response = future.result()
        except Exception as exc:
            return False, f'Failed to switch Servo command type: {exc}'

        if response is None or not response.success:
            return False, 'Servo rejected POSE command type.'
        return True, 'Servo command type is POSE.'

    def _wait_for_future_done(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while rclpy.ok():
            if future.done():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.02, remaining))
        return future.done()

    def _publish_feedback(
        self,
        goal_handle,
        point_index: int,
        total_points: int,
        message: str,
    ):
        feedback = FollowCartesianDelta.Feedback()
        feedback.current_point_index = int(point_index)
        feedback.total_points = int(total_points)
        feedback.message = message

        with self._state_lock:
            if self._latest_tcp_pose is not None:
                feedback.tcp_pose = copy.deepcopy(self._latest_tcp_pose)
            if self._latest_gripper_state is not None:
                feedback.gripper_state = copy.deepcopy(
                    self._latest_gripper_state
                )

        goal_handle.publish_feedback(feedback)

    def _set_status(
        self,
        busy: bool,
        active_point_index: int,
        active_point_count: int,
        status: str,
    ):
        with self._state_lock:
            self._busy = busy
            self._active_point_index = int(active_point_index)
            self._active_point_count = int(active_point_count)
            self._status = status

    def _finish_action(self, goal_handle, finish_state: str, message: str):
        result = FollowCartesianDelta.Result()
        result.success = finish_state == 'succeed'
        result.message = message

        with self._state_lock:
            if self._latest_tcp_pose is not None:
                result.final_tcp_pose = copy.deepcopy(self._latest_tcp_pose)
            if self._latest_gripper_state is not None:
                result.final_gripper_state = copy.deepcopy(
                    self._latest_gripper_state
                )

        if finish_state == 'succeed':
            goal_handle.succeed()
        elif finish_state == 'cancel':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def destroy_node(self):
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = URCartesianDeltaControl()
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
