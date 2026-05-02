import numpy as np


class RTDEArm:
    def __init__(
        self,
        robot_ip: str,
        servo_time: float,
        max_velocity: float,
        max_acceleration: float,
        lookahead_time: float,
        servo_gain: float,
    ):
        try:
            import rtde_control
            import rtde_receive
        except ImportError as exc:
            raise RuntimeError(
                "ur_rtde is not importable. Install/source the environment that provides "
                "rtde_control and rtde_receive."
            ) from exc

        self.robot_ip = robot_ip
        self.servo_time = float(servo_time)
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.lookahead_time = float(lookahead_time)
        self.servo_gain = float(servo_gain)
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

    def get_current_joints(self) -> np.ndarray:
        return np.array(self.rtde_r.getActualQ(), dtype=np.float64)

    def get_current_tcp_pose(self) -> np.ndarray:
        return np.array(self.rtde_r.getActualTCPPose(), dtype=np.float64)

    def inverse_kinematics(self, tcp_pose, q_near=None) -> np.ndarray | None:
        pose = np.array(tcp_pose, dtype=np.float64).tolist()
        qnear_list = None if q_near is None else np.array(q_near, dtype=np.float64).tolist()
        ik = getattr(self.rtde_c, "getInverseKinematics", None)
        if ik is None:
            return None

        attempts = []
        if qnear_list is not None:
            attempts.append((pose, qnear_list))
        attempts.append((pose,))

        for args in attempts:
            try:
                result = ik(*args)
            except TypeError:
                continue
            if result is None:
                continue
            result_array = np.array(result, dtype=np.float64)
            if result_array.size == 6 and np.all(np.isfinite(result_array)):
                return result_array
        return None

    def forward_kinematics(self, joint_positions) -> np.ndarray | None:
        fk = getattr(self.rtde_c, "getForwardKinematics", None)
        if fk is None:
            return None

        q = np.array(joint_positions, dtype=np.float64).tolist()
        attempts = [(q,), ()]
        for args in attempts:
            try:
                result = fk(*args)
            except TypeError:
                continue
            if result is None:
                continue
            result_array = np.array(result, dtype=np.float64)
            if result_array.size == 6 and np.all(np.isfinite(result_array)):
                return result_array
        return None

    def servo_joints(self, joint_positions):
        t_start = self.rtde_c.initPeriod()
        self.rtde_c.servoJ(
            np.array(joint_positions, dtype=np.float64).tolist(),
            self.max_velocity,
            self.max_acceleration,
            self.servo_time,
            self.lookahead_time,
            self.servo_gain,
        )
        self.rtde_c.waitPeriod(t_start)

    def servo_linear(self, tcp_pose):
        t_start = self.rtde_c.initPeriod()
        self.rtde_c.servoL(
            np.array(tcp_pose, dtype=np.float64).tolist(),
            self.max_velocity,
            self.max_acceleration,
            self.servo_time,
            self.lookahead_time,
            self.servo_gain,
        )
        self.rtde_c.waitPeriod(t_start)

    def stop_servo(self):
        try:
            self.rtde_c.servoStop()
        except Exception:
            pass

    def close(self):
        self.stop_servo()
        try:
            self.rtde_c.stopScript()
        except Exception:
            pass
        try:
            disconnect = getattr(self.rtde_r, "disconnect", None)
            if disconnect is not None:
                disconnect()
        except Exception:
            pass
