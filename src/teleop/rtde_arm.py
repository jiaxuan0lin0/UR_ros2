import socket
import time

import numpy as np


class RegisterSpeedJBridge:
    """Robot-side 2 ms speedJ loop fed by RTDE input integer registers."""

    ROBOT_SERVO_TIME = 0.002
    REGISTER_COUNT = 5
    COMMAND_MODE_RUN = 0
    COMMAND_MODE_EXIT = 2
    PAIR_OFFSET = 20000
    PAIR_BASE = 40001

    def __init__(
        self,
        robot_ip: str,
        rtde_io,
        register_start: int,
        velocity_scale: float,
        watchdog_sec: float,
        stale_stop_sec: float,
        upload_port: int,
        servo_time: float,
    ):
        self.robot_ip = robot_ip
        self.rtde_io = rtde_io
        self.register_start = int(register_start)
        self.registers = tuple(
            range(self.register_start, self.register_start + self.REGISTER_COUNT)
        )
        self.velocity_scale = float(velocity_scale)
        self.watchdog_sec = float(watchdog_sec)
        self.stale_stop_sec = float(stale_stop_sec)
        self.upload_port = int(upload_port)
        self.command_period = float(servo_time)
        self.sequence = 0
        self.started = False
        self.last_write_time = None

        if self.register_start not in (18, 42):
            raise ValueError(
                "speedj register_bridge expects --speedj-register-start 18 or 42"
            )
        if self.velocity_scale <= 0.0:
            raise ValueError("--speedj-velocity-scale must be positive")
        if self.watchdog_sec <= 0.0:
            raise ValueError("--speedj-watchdog-sec must be positive")
        if self.stale_stop_sec <= self.watchdog_sec:
            raise ValueError("--speedj-stale-stop-sec must be greater than watchdog")

    def start(self):
        if self.started:
            return
        self._write_command(np.zeros(6, dtype=np.float64), 1.0, self.COMMAND_MODE_RUN)
        self._upload_script(self._script())
        # Give the controller a moment to replace the previous RTDE control script.
        time.sleep(0.08)
        self.started = True
        print(
            "speedJ register bridge started: "
            f"registers={self.registers}, robot_cycle={self.ROBOT_SERVO_TIME:.4f}s, "
            f"python_update={self.command_period:.4f}s, "
            f"watchdog={self.watchdog_sec:.3f}s"
        )

    def send_velocity(self, joint_velocities, acceleration: float):
        if not self.started:
            self.start()
        elif self._might_have_gone_stale():
            self.started = False
            self.start()
        self._write_command(
            np.array(joint_velocities, dtype=np.float64),
            float(acceleration),
            self.COMMAND_MODE_RUN,
        )

    def send_zero(self, acceleration: float):
        if self.started:
            if self._might_have_gone_stale():
                self.started = False
                self.start()
            self._write_command(
                np.zeros(6, dtype=np.float64),
                float(acceleration),
                self.COMMAND_MODE_RUN,
            )

    def close(self, acceleration: float):
        if not self.started:
            self._disconnect_io()
            return
        try:
            self._write_command(
                np.zeros(6, dtype=np.float64),
                float(acceleration),
                self.COMMAND_MODE_EXIT,
            )
            time.sleep(0.05)
        finally:
            self.started = False
            self.last_write_time = None
            self._disconnect_io()

    def _write_command(self, joint_velocities: np.ndarray, acceleration: float, mode: int):
        values = np.array(joint_velocities, dtype=np.float64)
        if values.size != 6 or not np.all(np.isfinite(values)):
            values = np.zeros(6, dtype=np.float64)

        packed = (
            self._pack_pair(values[0], values[1]),
            self._pack_pair(values[2], values[3]),
            self._pack_pair(values[4], values[5]),
            self._pack_acceleration(acceleration),
            self._next_command(mode),
        )
        # Write the command register last so the robot never observes a new
        # sequence number before its corresponding qdot/acceleration registers.
        for register, value in zip(self.registers[:4], packed[:4], strict=True):
            self._write_register(register, value)
        self._write_register(self.registers[4], packed[4])
        self.last_write_time = time.monotonic()

    def _write_register(self, register: int, value: int):
        ok = self.rtde_io.setInputIntRegister(int(register), int(value))
        if not ok:
            raise RuntimeError(f"failed to write input_int_register_{register}")

    def _next_command(self, mode: int) -> int:
        self.sequence = (self.sequence + 1) % 100_000_000
        if self.sequence == 0:
            self.sequence = 1
        return self.sequence * 10 + int(mode)

    def _pack_pair(self, first: float, second: float) -> int:
        first_scaled = self._to_scaled_velocity(first)
        second_scaled = self._to_scaled_velocity(second)
        return int(
            (first_scaled + self.PAIR_OFFSET) * self.PAIR_BASE
            + (second_scaled + self.PAIR_OFFSET)
        )

    def _to_scaled_velocity(self, value: float) -> int:
        scaled = int(round(float(value) * self.velocity_scale))
        limit = self.PAIR_OFFSET - 1
        return int(np.clip(scaled, -limit, limit))

    def _pack_acceleration(self, acceleration: float) -> int:
        scaled = int(round(max(0.0, float(acceleration)) * self.velocity_scale))
        return int(np.clip(scaled, 0, 2_000_000))

    def _upload_script(self, script: str):
        with socket.create_connection(
            (self.robot_ip, self.upload_port),
            timeout=2.0,
        ) as sock:
            sock.settimeout(2.0)
            sock.sendall(script.encode("utf-8"))

    def _might_have_gone_stale(self) -> bool:
        if self.last_write_time is None:
            return False
        return (time.monotonic() - self.last_write_time) > (self.stale_stop_sec * 0.8)

    def _disconnect_io(self):
        try:
            disconnect = getattr(self.rtde_io, "disconnect", None)
            if disconnect is not None:
                disconnect()
        except Exception:
            pass

    def _script(self) -> str:
        r0, r1, r2, r_acc, r_cmd = self.registers
        scale = self.velocity_scale
        pair_base = self.PAIR_BASE
        pair_offset = self.PAIR_OFFSET
        servo_time = self.ROBOT_SERVO_TIME
        watchdog = max(servo_time * 2.0, self.watchdog_sec)
        stale_stop = max(watchdog + servo_time, self.stale_stop_sec)
        watchdog_ticks = max(1, int(round(watchdog / servo_time)))
        stale_stop_ticks = max(watchdog_ticks + 1, int(round(stale_stop / servo_time)))
        return f"""def teleop_speedj_register_bridge():
  textmsg("teleop speedj register bridge started")
  q0 = 0.0
  q1 = 0.0
  q2 = 0.0
  q3 = 0.0
  q4 = 0.0
  q5 = 0.0
	  bridge_acc = 1.0
	  last_cmd = read_input_integer_register({r_cmd})
	  stale_ticks = 0
	  watchdog_active = False
	  while True:
    command = read_input_integer_register({r_cmd})
    mode = command - floor(command / 10) * 10
    if mode == {self.COMMAND_MODE_EXIT}:
      break
    end
    if command != last_cmd:
      p0 = read_input_integer_register({r0})
      p1 = read_input_integer_register({r1})
      p2 = read_input_integer_register({r2})
      q0i = floor(p0 / {pair_base})
      q1i = p0 - q0i * {pair_base}
      q2i = floor(p1 / {pair_base})
      q3i = p1 - q2i * {pair_base}
      q4i = floor(p2 / {pair_base})
      q5i = p2 - q4i * {pair_base}
      q0i = q0i - {pair_offset}
      q1i = q1i - {pair_offset}
      q2i = q2i - {pair_offset}
      q3i = q3i - {pair_offset}
      q4i = q4i - {pair_offset}
      q5i = q5i - {pair_offset}
      q0 = q0i / {scale:.1f}
      q1 = q1i / {scale:.1f}
      q2 = q2i / {scale:.1f}
      q3 = q3i / {scale:.1f}
      q4 = q4i / {scale:.1f}
      q5 = q5i / {scale:.1f}
      bridge_acc = read_input_integer_register({r_acc}) / {scale:.1f}
      if bridge_acc < 0.1:
        bridge_acc = 0.1
      end
      if bridge_acc > 20.0:
        bridge_acc = 20.0
      end
	      last_cmd = command
	      stale_ticks = 0
	      watchdog_active = False
	    else:
	      stale_ticks = stale_ticks + 1
	    end
    if stale_ticks > {stale_stop_ticks}:
      break
    end
	    if stale_ticks > {watchdog_ticks}:
	      if watchdog_active == False:
	        textmsg("teleop speedj register bridge watchdog zero")
	        watchdog_active = True
	      end
	      speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], bridge_acc, {servo_time:.6f})
	    else:
	      watchdog_active = False
	      speedj([q0, q1, q2, q3, q4, q5], bridge_acc, {servo_time:.6f})
    end
  end
  speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], bridge_acc, 0.02)
  stopj(bridge_acc)
  textmsg("teleop speedj register bridge stopped")
end
teleop_speedj_register_bridge()
"""


class RTDEArm:
    def __init__(
        self,
        robot_ip: str,
        servo_time: float,
        control_frequency: float,
        max_velocity: float,
        max_acceleration: float,
        lookahead_time: float,
        servo_gain: float,
        speedj_backend: str = "direct",
        speedj_register_start: int = 42,
        speedj_velocity_scale: float = 10000.0,
        speedj_watchdog_sec: float = 0.06,
        speedj_stale_stop_sec: float = 1.0,
        speedj_upload_port: int = 30001,
        speed_slider_fraction: float = 1.0,
    ):
        try:
            import rtde_control
            import rtde_receive
            import rtde_io
        except ImportError as exc:
            raise RuntimeError(
                "ur_rtde is not importable. Install/source the environment that provides "
                "rtde_control, rtde_receive, and rtde_io."
            ) from exc

        self.robot_ip = robot_ip
        self.servo_time = float(servo_time)
        self.control_frequency = float(control_frequency)
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.lookahead_time = float(lookahead_time)
        self.servo_gain = float(servo_gain)
        self.speedj_backend = str(speedj_backend)
        self.speed_slider_fraction = float(speed_slider_fraction)
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip, self.control_frequency)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip, self.control_frequency)
        self.last_jacobian_error = None
        self.last_forward_kinematics_error = None
        self._next_bridge_cycle_time = None
        self._speed_bridge = None
        if self.speedj_backend == "register_bridge":
            use_upper_range = int(speedj_register_start) >= 42
            rtde_io_interface = rtde_io.RTDEIOInterface(
                robot_ip,
                False,
                use_upper_range,
            )
            self._speed_bridge = RegisterSpeedJBridge(
                robot_ip=robot_ip,
                rtde_io=rtde_io_interface,
                register_start=speedj_register_start,
                velocity_scale=speedj_velocity_scale,
                watchdog_sec=speedj_watchdog_sec,
                stale_stop_sec=speedj_stale_stop_sec,
                upload_port=speedj_upload_port,
                servo_time=self.servo_time,
            )
        elif self.speedj_backend != "direct":
            raise ValueError(f"unsupported speedj backend: {self.speedj_backend}")

    def start_speedj_backend(self):
        if self._speed_bridge is not None:
            try:
                if self.rtde_c.isProgramRunning():
                    self.rtde_c.stopScript()
            except Exception:
                pass
            try:
                self.rtde_c.disconnect()
            except Exception:
                pass
            time.sleep(0.30)
            self._set_speed_slider()
            time.sleep(0.20)
            self._speed_bridge.start()

    def _set_speed_slider(self):
        fraction = min(1.0, max(0.0, self.speed_slider_fraction))
        if fraction <= 0.0:
            return
        try:
            ok = self._speed_bridge.rtde_io.setSpeedSlider(fraction)
        except Exception as exc:
            print(f"Could not set UR speed slider: {exc}")
            return
        if not ok:
            print("Could not set UR speed slider: RTDEIO returned false.")

    def get_current_joints(self) -> np.ndarray:
        return np.array(self.rtde_r.getActualQ(), dtype=np.float64)

    def get_current_tcp_pose(self) -> np.ndarray:
        return np.array(self.rtde_r.getActualTCPPose(), dtype=np.float64)

    def get_tcp_offset(self) -> np.ndarray | None:
        get_tcp_offset = getattr(self.rtde_c, "getTCPOffset", None)
        if get_tcp_offset is None:
            return None
        try:
            result = get_tcp_offset()
        except Exception:
            return None
        offset = np.array(result, dtype=np.float64)
        if offset.size != 6 or not np.all(np.isfinite(offset)):
            return None
        return offset

    def inverse_kinematics(
        self,
        tcp_pose,
        q_near=None,
        max_position_error: float = 1.0e-4,
        max_orientation_error: float = 1.0e-3,
    ) -> np.ndarray | None:
        pose = np.array(tcp_pose, dtype=np.float64).tolist()
        qnear_list = None if q_near is None else np.array(q_near, dtype=np.float64).tolist()
        ik = getattr(self.rtde_c, "getInverseKinematics", None)
        if ik is None:
            return None

        attempts = []
        if qnear_list is not None:
            attempts.append(
                (
                    pose,
                    qnear_list,
                    float(max_position_error),
                    float(max_orientation_error),
                )
            )
            attempts.append((pose, qnear_list))
        attempts.append((pose, [], float(max_position_error), float(max_orientation_error)))
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

    def get_jacobian(self, joint_positions=None) -> np.ndarray | None:
        jacobian = getattr(self.rtde_c, "getJacobian", None)
        if jacobian is None:
            return None

        if joint_positions is None:
            attempts = [()]
        else:
            q = np.array(joint_positions, dtype=np.float64).tolist()
            attempts = [(q,), ()]

        for args in attempts:
            try:
                result = jacobian(*args)
            except TypeError:
                continue
            except Exception as exc:
                self.last_jacobian_error = str(exc)
                return None
            if result is None:
                continue
            result_array = np.array(result, dtype=np.float64)
            if result_array.size == 36 and np.all(np.isfinite(result_array)):
                self.last_jacobian_error = None
                return result_array.reshape((6, 6))
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
            except Exception as exc:
                self.last_forward_kinematics_error = str(exc)
                return None
            if result is None:
                continue
            result_array = np.array(result, dtype=np.float64)
            if result_array.size == 6 and np.all(np.isfinite(result_array)):
                self.last_forward_kinematics_error = None
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

    def speed_joints(self, joint_velocities, acceleration: float):
        if self._speed_bridge is not None:
            self._speed_bridge.send_velocity(joint_velocities, acceleration)
            self._wait_bridge_period()
            return

        t_start = self.rtde_c.initPeriod()
        self.rtde_c.speedJ(
            np.array(joint_velocities, dtype=np.float64).tolist(),
            float(acceleration),
            self.servo_time,
        )
        self.rtde_c.waitPeriod(t_start)

    def _wait_bridge_period(self):
        now = time.monotonic()
        if self._next_bridge_cycle_time is None:
            self._next_bridge_cycle_time = now + self.servo_time
        else:
            self._next_bridge_cycle_time += self.servo_time

        sleep_sec = self._next_bridge_cycle_time - time.monotonic()
        if sleep_sec > 0.0:
            time.sleep(sleep_sec)
        elif sleep_sec < -self.servo_time:
            self._next_bridge_cycle_time = time.monotonic()

    def stop_servo(self):
        try:
            self.rtde_c.servoStop()
        except Exception:
            pass

    def stop_speed(self, acceleration: float = 2.0):
        if self._speed_bridge is not None:
            try:
                self._speed_bridge.send_zero(float(acceleration))
            except Exception:
                pass
            self._next_bridge_cycle_time = None
            return
        try:
            self.rtde_c.speedStop(float(acceleration))
        except Exception:
            pass

    def keep_speedj_backend_alive(self, acceleration: float = 2.0):
        if self._speed_bridge is None:
            return
        try:
            self._speed_bridge.send_zero(float(acceleration))
        except Exception:
            pass

    def close(self):
        if self._speed_bridge is not None:
            self._speed_bridge.close(self.max_acceleration)
        self.stop_speed()
        self.stop_servo()
        try:
            if self.rtde_c.isConnected():
                self.rtde_c.stopScript()
        except Exception:
            pass
        try:
            disconnect = getattr(self.rtde_r, "disconnect", None)
            if disconnect is not None:
                disconnect()
        except Exception:
            pass
