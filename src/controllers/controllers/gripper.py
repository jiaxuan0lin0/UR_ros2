import time

import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from controllers.dh_gripper import dh_modbus_gripper


class Gripper(Node):
    MIN_FORCE = 20
    MAX_FORCE = 100
    MIN_SPEED = 1
    MAX_SPEED = 100
    MIN_POSITION = 0
    MAX_POSITION = 1000
    RECONNECT_INTERVAL_SEC = 1.0
    INIT_TIMEOUT_SEC = 8.0
    INIT_POLL_INTERVAL_SEC = 0.2

    def __init__(self):
        super().__init__('gripper')

        self.declare_parameter('port', '/tmp/ttyUR')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.gripper = dh_modbus_gripper.dh_modbus_gripper()
        self.gripper_ready = False

        self.command_queue = deque()
        self.active_command = None
        self.exec_timer = self.create_timer(0.02, self.execute_step)
        self.reconnect_timer = self.create_timer(
            self.RECONNECT_INTERVAL_SEC,
            self.ensure_gripper_ready,
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/gripper_joint_states',
            10
        )
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/gripper_trajectory',
            self.trajectory_cb,
            10,
        )
        self.state_timer = self.create_timer(0.05, self.publish_joint_state)
        self.ensure_gripper_ready()

    def safe_close_gripper(self):
        try:
            self.gripper.close()
        except Exception:
            pass

    def handle_gripper_disconnect(self, reason):
        if self.gripper_ready:
            self.get_logger().warn(reason)
        self.gripper_ready = False
        self.active_command = None
        self.command_queue.clear()
        self.safe_close_gripper()

    def ensure_gripper_ready(self):
        if self.gripper_ready:
            return

        try:
            ret = self.gripper.open(self.port, self.baudrate)
            if ret < 0:
                self.get_logger().warn(
                    f'failed to open gripper port {self.port}, will retry'
                )
                return

            self.gripper.Initialization()
            deadline = time.monotonic() + self.INIT_TIMEOUT_SEC
            init_state = 0

            while time.monotonic() < deadline:
                init_state = self.gripper.GetInitState()
                if init_state == 1:
                    self.gripper_ready = True
                    self.get_logger().info(
                        f'gripper connected on {self.port} @ {self.baudrate}'
                    )
                    return
                time.sleep(self.INIT_POLL_INTERVAL_SEC)

            self.get_logger().warn('gripper init timeout, will retry')
            self.safe_close_gripper()
        except Exception as exc:
            self.safe_close_gripper()
            self.get_logger().warn(
                f'gripper connect failed on {self.port}: {exc}, will retry'
            )

    def clamp_command(self, value, min_value, max_value, name):
        int_value = int(round(value))
        clamped_value = max(min_value, min(int_value, max_value))

        if clamped_value != int_value:
            self.get_logger().warn(
                f'{name} command {value} out of range, clamped to {clamped_value}'
            )

        return clamped_value

    def publish_joint_state(self): 
        if not self.gripper_ready:
            return

        current_position = self.gripper.GetCurrentPosition()
        current_speed = self.gripper.GetCurrentTargetSpeed()
        current_force = self.gripper.GetCurrentTargetForce()

        if current_position is None:
            self.handle_gripper_disconnect('failed to read gripper position')
            return
        if current_speed is None:
            self.handle_gripper_disconnect('failed to read gripper speed')
            return
        if current_force is None:
            self.handle_gripper_disconnect('failed to read gripper force')
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper_joint']
        msg.position = [float(current_position)]
        msg.velocity = [float(current_speed)]
        msg.effort = [float(current_force)]

        self.joint_state_pub.publish(msg)

    def trajectory_cb(self, msg: JointTrajectory):
        if not self.gripper_ready:
            self.get_logger().warn('gripper not ready, ignore trajectory')
            return

        if self.active_command is not None or self.command_queue:
            self.get_logger().warn('gripper is busy, ignore new trajectory')
            return

        if not msg.points:
            self.get_logger().warn('received empty trajectory')
            return

        for point in msg.points:
            raw_position = point.positions[0] if point.positions else 0.0
            raw_velocity = point.velocities[0] if point.velocities else 100.0
            raw_effort = point.effort[0] if point.effort else 100.0

            command = {
                'position': self.clamp_command(raw_position, self.MIN_POSITION, self.MAX_POSITION, 'position'),
                'velocity': self.clamp_command(raw_velocity, self.MIN_SPEED, self.MAX_SPEED, 'velocity'),
                'effort': self.clamp_command(raw_effort, self.MIN_FORCE, self.MAX_FORCE, 'effort'),
            }
            self.command_queue.append(command)

        self.get_logger().info(f'queued {len(self.command_queue)} trajectory points')

        
    def execute_step(self):
        if not self.gripper_ready:
            return

        if self.active_command is None:
            if not self.command_queue:
                return
            
            self.active_command = self.command_queue.popleft()
            try:
                self.gripper.SetTargetForce(self.active_command['effort'])
                self.gripper.SetTargetSpeed(self.active_command['velocity'])
                self.gripper.SetTargetPosition(self.active_command['position'])
            except Exception as exc:
                self.handle_gripper_disconnect(
                    f'failed to send gripper command: {exc}'
                )
                return

            self.get_logger().info(
                f"send command pos={self.active_command['position']} "
                f"vel={self.active_command['velocity']} "
                f"eff={self.active_command['effort']}"
            )
            return

        gripper_state = self.gripper.GetGripState()
        if gripper_state is None:
            self.handle_gripper_disconnect('failed to read gripper state')
            return
        if gripper_state in (1, 2, 3):
            self.get_logger().info(f'gripper motion finished, state={gripper_state}')
            self.active_command = None

def main(args=None):
    rclpy.init(args=args)
    node = Gripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
