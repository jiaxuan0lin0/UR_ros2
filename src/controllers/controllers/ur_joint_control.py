import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
GRIPPER_JOINT_NAME = 'gripper_joint'


class URJointControl(Node):
    """Top-level 7-DoF joint-control facade for UR arm + DH gripper."""

    def __init__(self):
        super().__init__('ur_joint_control')
        self.callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._gripper_plan_cv = threading.Condition()

        self.declare_parameter('arm_joint_state_topic', '/joint_states')
        self.declare_parameter(
            'gripper_joint_state_topic',
            '/gripper_joint_states',
        )
        self.declare_parameter(
            'joint_state_topic',
            '/ur/joint_states',
        )
        self.declare_parameter(
            'joint_control_action_name',
            '/ur/joint_control/follow_joint_trajectory',
        )
        self.declare_parameter(
            'arm_action_name',
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
        )
        self.declare_parameter(
            'gripper_trajectory_topic',
            '/gripper_trajectory',
        )
        self.declare_parameter('state_publish_hz', 50.0)
        self.declare_parameter('gripper_raw_min', 0.0)
        self.declare_parameter('gripper_raw_max', 1000.0)
        self.declare_parameter('gripper_default_speed', 100.0)
        self.declare_parameter('gripper_default_force', 100.0)
        self.declare_parameter('arm_goal_response_timeout_sec', 2.0)
        self.declare_parameter('execution_timeout_margin_sec', 2.0)
        self.declare_parameter('wait_poll_sec', 0.02)

        self.arm_joint_state_topic = self.get_parameter(
            'arm_joint_state_topic'
        ).value
        self.gripper_joint_state_topic = self.get_parameter(
            'gripper_joint_state_topic'
        ).value
        self.joint_state_topic = self.get_parameter(
            'joint_state_topic'
        ).value
        self.joint_control_action_name = self.get_parameter(
            'joint_control_action_name'
        ).value
        self.arm_action_name = self.get_parameter('arm_action_name').value
        self.gripper_trajectory_topic = self.get_parameter(
            'gripper_trajectory_topic'
        ).value
        self.state_publish_hz = float(
            self.get_parameter('state_publish_hz').value
        )
        self.gripper_raw_min = float(
            self.get_parameter('gripper_raw_min').value
        )
        self.gripper_raw_max = float(
            self.get_parameter('gripper_raw_max').value
        )
        self.gripper_default_speed = float(
            self.get_parameter('gripper_default_speed').value
        )
        self.gripper_default_force = float(
            self.get_parameter('gripper_default_force').value
        )
        self.arm_goal_response_timeout_sec = float(
            self.get_parameter('arm_goal_response_timeout_sec').value
        )
        self.execution_timeout_margin_sec = float(
            self.get_parameter('execution_timeout_margin_sec').value
        )
        self.wait_poll_sec = float(self.get_parameter('wait_poll_sec').value)

        self.joint_names = ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
        self._arm_position_by_name = {}
        self._arm_velocity_by_name = {}
        self._arm_effort_by_name = {}
        self._gripper_position_norm = None
        self._gripper_velocity_norm = None
        self._gripper_effort = None
        self._active_gripper_plan = None
        self._gripper_plan_seq = 0
        self._gripper_worker_stop = False
        self._execution_lock = threading.Lock()

        self.arm_joint_state_sub = self.create_subscription(
            JointState,
            self.arm_joint_state_topic,
            self._arm_joint_state_callback,
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
        self.joint_state_pub = self.create_publisher(
            JointState,
            self.joint_state_topic,
            10,
        )
        self.gripper_trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.gripper_trajectory_topic,
            10,
        )
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.arm_action_name,
            callback_group=self.callback_group,
        )
        self.joint_control_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.joint_control_action_name,
            execute_callback=self._execute_joint_control_action,
            callback_group=self.callback_group,
        )

        state_period_sec = 1.0 / max(self.state_publish_hz, 1.0)
        self.state_timer = self.create_timer(
            state_period_sec,
            self._publish_joint_state,
            callback_group=self.callback_group,
        )
        self._gripper_plan_thread = threading.Thread(
            target=self._gripper_plan_worker,
            daemon=True,
        )
        self._gripper_plan_thread.start()

        self.get_logger().info(
            'UR joint control ready: '
            f'state={self.joint_state_topic}, '
            f'joint_action={self.joint_control_action_name}'
        )

    def _arm_joint_state_callback(self, msg: JointState):
        position_map = dict(zip(msg.name, msg.position))
        velocity_map = (
            dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        )
        effort_map = dict(zip(msg.name, msg.effort)) if msg.effort else {}

        with self._state_lock:
            for name in ARM_JOINT_NAMES:
                if name in position_map:
                    self._arm_position_by_name[name] = float(
                        position_map[name]
                    )
                if name in velocity_map:
                    self._arm_velocity_by_name[name] = float(
                        velocity_map[name]
                    )
                if name in effort_map:
                    self._arm_effort_by_name[name] = float(effort_map[name])

    def _gripper_joint_state_callback(self, msg: JointState):
        if not msg.position:
            return

        try:
            index = msg.name.index(GRIPPER_JOINT_NAME) if msg.name else 0
        except ValueError:
            index = 0

        with self._state_lock:
            self._gripper_position_norm = self._raw_to_norm(
                msg.position[index]
            )
            if msg.velocity and index < len(msg.velocity):
                self._gripper_velocity_norm = self._raw_to_norm(
                    msg.velocity[index]
                )
            if msg.effort and index < len(msg.effort):
                self._gripper_effort = float(msg.effort[index])

    def _get_joint_position_locked(self):
        if self._gripper_position_norm is None:
            return None
        if any(
            name not in self._arm_position_by_name
            for name in ARM_JOINT_NAMES
        ):
            return None

        return [
            self._arm_position_by_name[name] for name in ARM_JOINT_NAMES
        ] + [self._gripper_position_norm]

    def _publish_joint_state(self):
        with self._state_lock:
            positions = self._get_joint_position_locked()
            if positions is None:
                return

            velocities = []
            if (
                self._gripper_velocity_norm is not None
                and all(
                    name in self._arm_velocity_by_name
                    for name in ARM_JOINT_NAMES
                )
            ):
                velocities = [
                    self._arm_velocity_by_name[name]
                    for name in ARM_JOINT_NAMES
                ] + [self._gripper_velocity_norm]

            efforts = []
            if (
                self._gripper_effort is not None
                and all(
                    name in self._arm_effort_by_name
                    for name in ARM_JOINT_NAMES
                )
            ):
                efforts = [
                    self._arm_effort_by_name[name] for name in ARM_JOINT_NAMES
                ] + [self._gripper_effort]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self.joint_state_pub.publish(msg)

    def _execute_joint_control_action(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            return self._abort_action(
                goal_handle,
                'UR joint control is busy.',
            )

        try:
            ok, status = self._dispatch_joint_trajectory(
                goal_handle.request.trajectory,
                wait=True,
            )
        finally:
            self._execution_lock.release()

        result = FollowJointTrajectory.Result()
        result.error_string = status
        if ok:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
        else:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
        return result

    @staticmethod
    def _abort_action(goal_handle, message: str):
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = message
        goal_handle.abort()
        return result

    def _dispatch_joint_trajectory(
        self,
        trajectory: JointTrajectory,
        wait: bool,
    ):
        if not trajectory.points:
            return False, 'Received empty 7-DoF trajectory.'

        indexes = self._resolve_joint_indexes(trajectory.joint_names)
        if indexes is None:
            return (
                False,
                '7-DoF trajectory joint_names must contain '
                f'{self.joint_names}.',
            )

        arm_goal = self._build_arm_goal(trajectory, indexes)
        if arm_goal is None:
            return False, 'Every 7-DoF trajectory point needs arm positions.'

        gripper_plan = self._build_gripper_plan(trajectory, indexes)
        if gripper_plan is None:
            return (
                False,
                'Every 7-DoF trajectory point needs gripper position.',
            )

        if not self.arm_action_client.wait_for_server(timeout_sec=0.0):
            if not self.arm_action_client.wait_for_server(
                timeout_sec=self.arm_goal_response_timeout_sec
            ):
                return (
                    False,
                    f'Arm action server not ready: {self.arm_action_name}',
                )

        send_goal_future = self.arm_action_client.send_goal_async(arm_goal)
        if not self._wait_for_future_done(
            send_goal_future,
            self.arm_goal_response_timeout_sec,
        ):
            return False, 'Timed out waiting for arm trajectory goal response.'

        try:
            goal_handle = send_goal_future.result()
        except Exception as exc:
            return False, f'Failed to send arm trajectory goal: {exc}'

        if goal_handle is None or not goal_handle.accepted:
            return False, 'Arm trajectory goal was rejected.'

        gripper_plan_seq = self._start_gripper_plan(
            gripper_plan,
            time.monotonic(),
        )
        result_future = goal_handle.get_result_async()
        if not wait:
            return True, '7-DoF trajectory dispatched.'

        execution_timeout_sec = self._trajectory_duration_sec(trajectory)
        execution_timeout_sec += self.execution_timeout_margin_sec

        arm_finished = self._wait_for_future_done(
            result_future,
            execution_timeout_sec,
        )
        gripper_finished = self._wait_for_gripper_plan(
            gripper_plan_seq,
            execution_timeout_sec,
        )

        if not arm_finished:
            return False, 'Timed out waiting for arm trajectory execution.'
        if not gripper_finished:
            return False, 'Timed out waiting for gripper plan execution.'

        try:
            result = result_future.result()
        except Exception as exc:
            return False, f'Failed to get arm trajectory result: {exc}'

        status = getattr(result, 'status', None)
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'Arm trajectory finished with status={status}.'
        return True, '7-DoF trajectory completed.'

    def _resolve_joint_indexes(self, joint_names):
        if not joint_names:
            return None

        name_to_index = {name: index for index, name in enumerate(joint_names)}
        if any(
            name not in name_to_index for name in self.joint_names
        ):
            return None

        return {
            name: name_to_index[name] for name in self.joint_names
        }

    def _build_arm_goal(self, trajectory: JointTrajectory, indexes):
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.header = trajectory.header
        arm_goal.trajectory.joint_names = list(ARM_JOINT_NAMES)

        arm_indexes = [indexes[name] for name in ARM_JOINT_NAMES]
        for point in trajectory.points:
            arm_point = JointTrajectoryPoint()
            arm_point.positions = self._select_values(
                point.positions,
                arm_indexes,
            )
            if len(arm_point.positions) != len(ARM_JOINT_NAMES):
                return None

            arm_point.velocities = self._select_values(
                point.velocities,
                arm_indexes,
            )
            arm_point.accelerations = self._select_values(
                point.accelerations,
                arm_indexes,
            )
            arm_point.effort = self._select_values(point.effort, arm_indexes)
            arm_point.time_from_start = point.time_from_start
            arm_goal.trajectory.points.append(arm_point)

        return arm_goal

    def _build_gripper_plan(self, trajectory: JointTrajectory, indexes):
        gripper_index = indexes[GRIPPER_JOINT_NAME]
        plan = []

        for point in trajectory.points:
            if len(point.positions) <= gripper_index:
                return None

            offset_sec = self._duration_to_sec(point.time_from_start)
            gripper_raw = self._norm_to_raw(point.positions[gripper_index])
            plan.append((offset_sec, gripper_raw))

        return plan

    def _start_gripper_plan(self, plan, start_time: float) -> int:
        with self._gripper_plan_cv:
            self._gripper_plan_seq += 1
            plan_seq = self._gripper_plan_seq
            self._active_gripper_plan = (plan_seq, start_time, plan)
            self._gripper_plan_cv.notify_all()
            return plan_seq

    def _gripper_plan_worker(self):
        while True:
            with self._gripper_plan_cv:
                while (
                    not self._gripper_worker_stop
                    and self._active_gripper_plan is None
                ):
                    self._gripper_plan_cv.wait(timeout=0.2)
                if self._gripper_worker_stop:
                    return
                plan_seq, start_time, plan = self._active_gripper_plan

            for offset_sec, target_raw in plan:
                while True:
                    with self._gripper_plan_cv:
                        if self._gripper_worker_stop:
                            return
                        active_plan = self._active_gripper_plan
                    if active_plan is None or active_plan[0] != plan_seq:
                        break

                    due_time = start_time + offset_sec
                    now = time.monotonic()
                    if now < due_time:
                        time.sleep(min(due_time - now, self.wait_poll_sec))
                        continue

                    self._publish_gripper_target(target_raw)
                    break

                with self._gripper_plan_cv:
                    active_plan = self._active_gripper_plan
                if active_plan is None or active_plan[0] != plan_seq:
                    break

            with self._gripper_plan_cv:
                if (
                    self._active_gripper_plan is not None
                    and self._active_gripper_plan[0] == plan_seq
                ):
                    self._active_gripper_plan = None
                    self._gripper_plan_cv.notify_all()

    def _publish_gripper_target(self, target_raw: float):
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [GRIPPER_JOINT_NAME]

        point = JointTrajectoryPoint()
        point.positions = [float(target_raw)]
        point.velocities = [self.gripper_default_speed]
        point.effort = [self.gripper_default_force]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0
        trajectory.points.append(point)

        self.gripper_trajectory_pub.publish(trajectory)

    def _wait_for_gripper_plan(
        self,
        plan_seq: int,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        with self._gripper_plan_cv:
            while not self._gripper_worker_stop:
                active_plan = self._active_gripper_plan
                if active_plan is None or active_plan[0] != plan_seq:
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._gripper_plan_cv.wait(
                    timeout=min(self.wait_poll_sec, remaining)
                )
        return False

    @staticmethod
    def _select_values(values, indexes):
        if not values:
            return []
        if len(values) <= max(indexes):
            return []
        return [float(values[index]) for index in indexes]

    @staticmethod
    def _duration_to_sec(duration) -> float:
        return float(duration.sec) + float(duration.nanosec) / 1e9

    def _trajectory_duration_sec(self, trajectory: JointTrajectory) -> float:
        return max(
            (self._duration_to_sec(point.time_from_start)
             for point in trajectory.points),
            default=0.0,
        )

    def _raw_to_norm(self, value: float) -> float:
        raw_range = self.gripper_raw_max - self.gripper_raw_min
        if raw_range <= 0.0:
            return 0.0
        clipped = min(
            self.gripper_raw_max,
            max(self.gripper_raw_min, float(value)),
        )
        return (clipped - self.gripper_raw_min) / raw_range

    def _norm_to_raw(self, value: float) -> float:
        raw_range = self.gripper_raw_max - self.gripper_raw_min
        clipped = min(1.0, max(0.0, float(value)))
        return clipped * raw_range + self.gripper_raw_min

    def _wait_for_future_done(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while rclpy.ok():
            if future.done():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(self.wait_poll_sec, remaining))
        return future.done()

    def destroy_node(self):
        with self._gripper_plan_cv:
            self._gripper_worker_stop = True
            self._active_gripper_plan = None
            self._gripper_plan_cv.notify_all()
        if self._gripper_plan_thread.is_alive():
            self._gripper_plan_thread.join(timeout=1.0)
        self.joint_control_action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = URJointControl()
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
