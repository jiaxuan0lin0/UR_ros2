#!/usr/bin/env python3
import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.config import (
    ArmConfig,
    GripperConfig,
    RecorderControlConfig,
    ReturnHomeConfig,
    SafetyConfig,
    StateConfig,
    TeleopConfig,
    XrConfig,
)
from teleop.gripper_direct import DirectDHGripper
from teleop.pose_mapper import PoseMapper
from teleop.recorder_control import RecorderControl
from teleop.rtde_arm import RTDEArm
from teleop.safety import SafetyLimiter
from teleop.state_writer import StateWriter
from teleop.xr_input import XrInput


def parse_xyz(value: str):
    if value == "":
        return None
    parts = [float(item.strip()) for item in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return tuple(parts)


def parse_joint_positions(value: str):
    parts = [float(item.strip()) for item in value.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("expected q1,q2,q3,q4,q5,q6")
    return tuple(parts)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="High-performance XR RTDE teleoperation.")
    parser.add_argument("--robot-ip", default="192.168.1.102")
    parser.add_argument("--control-mode", choices=("servo_j", "servo_l"), default="servo_j")
    parser.add_argument("--rate-hz", type=float, default=250.0)
    parser.add_argument("--max-velocity", type=float, default=0.5)
    parser.add_argument("--max-acceleration", type=float, default=1.0)
    parser.add_argument("--lookahead-time", type=float, default=0.06)
    parser.add_argument("--servo-gain", type=float, default=500.0)

    parser.add_argument("--pose-source", default="right_controller")
    parser.add_argument("--grip-name", default="right_grip")
    parser.add_argument("--trigger-name", default="right_trigger")
    parser.add_argument("--emergency-button", default="right_axis_click")
    parser.add_argument("--grip-threshold", type=float, default=0.95)
    parser.add_argument("--grip-release-threshold", type=float, default=None)
    parser.add_argument("--xr-timeout-sec", type=float, default=0.2)
    parser.add_argument("--scale-factor", type=float, default=0.6)
    parser.add_argument("--rotation-scale", type=float, default=1.0)

    parser.add_argument("--max-step-translation", type=float, default=0.01)
    parser.add_argument("--max-step-rotation", type=float, default=0.05)
    parser.add_argument("--max-joint-step", type=float, default=0.04)
    parser.add_argument("--workspace-min", type=parse_xyz, default=None)
    parser.add_argument("--workspace-max", type=parse_xyz, default=None)

    parser.add_argument("--disable-gripper", action="store_true")
    parser.add_argument("--gripper-port", default="/tmp/ttyUR")
    parser.add_argument("--gripper-baudrate", type=int, default=115200)
    parser.add_argument("--gripper-rate-hz", type=float, default=30.0)
    parser.add_argument("--gripper-speed", type=int, default=100)
    parser.add_argument("--gripper-force", type=int, default=50)
    parser.add_argument("--gripper-open-raw", type=int, default=1000)
    parser.add_argument("--gripper-closed-raw", type=int, default=0)
    parser.add_argument("--gripper-binary-control", action="store_true")
    parser.add_argument("--gripper-trigger-threshold", type=float, default=0.5)
    parser.add_argument("--gripper-deadband", type=int, default=4)
    parser.add_argument("--gripper-init-timeout-sec", type=float, default=8.0)
    parser.add_argument("--gripper-read-hz", type=float, default=5.0)

    parser.add_argument("--disable-state-file", action="store_true")
    parser.add_argument("--state-file", type=Path, default=Path("/tmp/ur_teleop_state.json"))
    parser.add_argument("--state-hz", type=float, default=30.0)

    parser.add_argument("--enable-recorder-control", action="store_true")
    parser.add_argument("--recorder-start-button", default="A")
    parser.add_argument("--recorder-stop-button", default="B")
    parser.add_argument("--recorder-save-button", default="X")
    parser.add_argument("--recorder-discard-button", default="Y")
    parser.add_argument("--recorder-debounce-sec", type=float, default=0.5)

    parser.add_argument("--disable-return-home-before-stop", action="store_true")
    parser.add_argument("--return-home-joints", type=parse_joint_positions, default=None)
    parser.add_argument("--return-home-max-joint-step", type=float, default=0.0006)
    parser.add_argument("--return-home-tolerance", type=float, default=0.01)
    parser.add_argument("--return-home-timeout-sec", type=float, default=20.0)
    parser.add_argument("--return-home-hold-sec", type=float, default=0.3)
    return parser


def config_from_args(args) -> TeleopConfig:
    arm = ArmConfig(
        robot_ip=args.robot_ip,
        control_mode=args.control_mode,
        rate_hz=args.rate_hz,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        lookahead_time=args.lookahead_time,
        servo_gain=args.servo_gain,
    )
    grip_threshold = min(1.0, max(0.0, float(args.grip_threshold)))
    if args.grip_release_threshold is None:
        grip_release_threshold = max(0.0, grip_threshold - 0.15)
    else:
        grip_release_threshold = float(args.grip_release_threshold)
    grip_release_threshold = min(grip_threshold, max(0.0, grip_release_threshold))

    xr = XrConfig(
        pose_source=args.pose_source,
        grip_name=args.grip_name,
        trigger_name=args.trigger_name,
        emergency_button=args.emergency_button,
        grip_threshold=grip_threshold,
        grip_release_threshold=grip_release_threshold,
        timeout_sec=args.xr_timeout_sec,
        scale_factor=args.scale_factor,
        rotation_scale=args.rotation_scale,
    )
    safety = SafetyConfig(
        max_step_translation=args.max_step_translation,
        max_step_rotation=args.max_step_rotation,
        max_joint_step=args.max_joint_step,
        workspace_min=args.workspace_min,
        workspace_max=args.workspace_max,
    )
    gripper = GripperConfig(
        enabled=not args.disable_gripper,
        port=args.gripper_port,
        baudrate=args.gripper_baudrate,
        rate_hz=args.gripper_rate_hz,
        speed=args.gripper_speed,
        force=args.gripper_force,
        open_raw=args.gripper_open_raw,
        closed_raw=args.gripper_closed_raw,
        binary_control=args.gripper_binary_control,
        trigger_threshold=args.gripper_trigger_threshold,
        position_deadband=args.gripper_deadband,
        init_timeout_sec=args.gripper_init_timeout_sec,
        read_state_hz=args.gripper_read_hz,
    )
    state = StateConfig(
        enabled=not args.disable_state_file,
        path=args.state_file,
        rate_hz=args.state_hz,
    )
    recorder = RecorderControlConfig(
        enabled=args.enable_recorder_control,
        start_button=args.recorder_start_button,
        stop_button=args.recorder_stop_button,
        save_button=args.recorder_save_button,
        discard_button=args.recorder_discard_button,
        debounce_sec=args.recorder_debounce_sec,
    )
    return_home = ReturnHomeConfig(
        enabled=not args.disable_return_home_before_stop,
        joint_positions=(
            args.return_home_joints
            if args.return_home_joints is not None
            else ReturnHomeConfig().joint_positions
        ),
        max_joint_step=args.return_home_max_joint_step,
        tolerance=args.return_home_tolerance,
        timeout_sec=args.return_home_timeout_sec,
        hold_sec=args.return_home_hold_sec,
    )
    return TeleopConfig(
        arm=arm,
        xr=xr,
        safety=safety,
        gripper=gripper,
        state=state,
        recorder=recorder,
        return_home=return_home,
    )


class TeleopRunner:
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.stop_requested = False
        self.xr = XrInput()
        self.arm = RTDEArm(
            robot_ip=config.arm.robot_ip,
            servo_time=1.0 / max(config.arm.rate_hz, 1.0),
            max_velocity=config.arm.max_velocity,
            max_acceleration=config.arm.max_acceleration,
            lookahead_time=config.arm.lookahead_time,
            servo_gain=config.arm.servo_gain,
        )
        self.mapper = PoseMapper(
            scale_factor=config.xr.scale_factor,
            rotation_scale=config.xr.rotation_scale,
        )
        self.safety = SafetyLimiter(
            max_step_translation=config.safety.max_step_translation,
            max_step_rotation=config.safety.max_step_rotation,
            max_joint_step=config.safety.max_joint_step,
            workspace_min=config.safety.workspace_min,
            workspace_max=config.safety.workspace_max,
        )
        self.gripper = None
        if config.gripper.enabled:
            self.gripper = DirectDHGripper(
                port=config.gripper.port,
                baudrate=config.gripper.baudrate,
                rate_hz=config.gripper.rate_hz,
                speed=config.gripper.speed,
                force=config.gripper.force,
                position_deadband=config.gripper.position_deadband,
                init_timeout_sec=config.gripper.init_timeout_sec,
                read_state_hz=config.gripper.read_state_hz,
            )
            self.gripper.set_target(config.gripper.open_raw)
        self.state_writer = (
            StateWriter(config.state.path, config.state.rate_hz)
            if config.state.enabled
            else None
        )
        self.recorder_control = None
        if config.recorder.enabled:
            self.recorder_control = RecorderControl(
                button_map={
                    "start": config.recorder.start_button,
                    "save": config.recorder.save_button,
                    "discard": config.recorder.discard_button,
                },
                debounce_sec=config.recorder.debounce_sec,
            )
        self.return_home_target_q = np.array(
            config.return_home.joint_positions,
            dtype=np.float64,
        )
        self.return_home_active = False
        self.return_home_started_at = None
        self.return_home_reached_since = None
        self._stop_button_was_pressed = False
        self._last_stop_button_time = 0.0
        self._motion_locked_until_grip_release = False
        self.previous_target_tcp = None
        self.previous_target_q = None
        self.enabled = False
        self._grip_active = False
        self._last_warn_time = 0.0
        self._last_xr_timestamp = None
        self._last_xr_update_wall = time.monotonic()

    def request_stop(self, *_args):
        self.stop_requested = True

    def _warn_throttled(self, message: str, interval_sec: float = 1.0):
        now = time.monotonic()
        if now - self._last_warn_time >= interval_sec:
            print(message)
            self._last_warn_time = now

    def _trigger_to_raw(self, trigger_value: float) -> int:
        clipped = min(1.0, max(0.0, float(trigger_value)))
        open_raw = self.config.gripper.open_raw
        closed_raw = self.config.gripper.closed_raw
        if self.config.gripper.binary_control:
            threshold = min(
                1.0,
                max(0.0, float(self.config.gripper.trigger_threshold)),
            )
            return closed_raw if clipped >= threshold else open_raw
        return int(round(open_raw + (closed_raw - open_raw) * clipped))

    def _handle_gripper(self):
        if self.gripper is None:
            return None
        trigger = self.xr.get_value(self.config.xr.trigger_name)
        raw = self._trigger_to_raw(trigger)
        self.gripper.set_target(raw)
        return raw

    def _motion_enabled_by_grip(self, grip_value: float) -> bool:
        grip = min(1.0, max(0.0, float(grip_value)))
        if self._grip_active:
            self._grip_active = grip >= self.config.xr.grip_release_threshold
        else:
            self._grip_active = grip >= self.config.xr.grip_threshold
        return self._grip_active

    def _handle_recorder_control(self):
        if self.recorder_control is None:
            return
        self._handle_recorder_stop_button()
        self.recorder_control.poll(self.xr.get_button)

    def _handle_recorder_stop_button(self):
        button = self.config.recorder.stop_button
        pressed = bool(self.xr.get_button(button))
        was_pressed = self._stop_button_was_pressed
        self._stop_button_was_pressed = pressed
        if not pressed or was_pressed:
            return

        now = time.monotonic()
        if now - self._last_stop_button_time < self.config.recorder.debounce_sec:
            return
        self._last_stop_button_time = now

        if self.config.return_home.enabled:
            self._start_return_home()
        else:
            self.recorder_control.request("stop")

    def _start_return_home(self):
        if self.return_home_active:
            print("Return-home is already running.")
            return

        if self.enabled:
            self._deactivate()

        self.previous_target_q = self.arm.get_current_joints()
        self.previous_target_tcp = self.arm.get_current_tcp_pose()
        self.return_home_active = True
        self.return_home_started_at = time.monotonic()
        self.return_home_reached_since = None
        print("Return-home started; recorder will stop after home is reached.")

    def _write_state(self, _status: str, gripper_raw, target_tcp=None, target_q=None):
        if self.state_writer is None:
            return
        gripper_current_raw = self.gripper.get_current() if self.gripper is not None else None
        state = {
            "gripper_target_raw": gripper_raw,
            "gripper_current_raw": gripper_current_raw,
        }
        if target_tcp is not None:
            state["target_tcp_pose"] = np.array(target_tcp, dtype=float).tolist()
        if target_q is not None:
            state["target_q"] = np.array(target_q, dtype=float).tolist()
        self.state_writer.maybe_write(state)

    def _xr_is_fresh(self) -> bool:
        timestamp = self.xr.timestamp_sec()
        if timestamp <= 0.0:
            return True
        now = time.monotonic()
        if self._last_xr_timestamp is None or timestamp != self._last_xr_timestamp:
            self._last_xr_timestamp = timestamp
            self._last_xr_update_wall = now
            return True
        return (now - self._last_xr_update_wall) <= self.config.xr.timeout_sec

    def _deactivate(self):
        if self.enabled:
            print("Teleop disabled.")
            self.arm.stop_servo()
        self.enabled = False
        self.mapper.reset()
        self.previous_target_tcp = None
        self.previous_target_q = None

    def _activate(self, xr_pose):
        tcp_pose = self.arm.get_current_tcp_pose()
        self.mapper.begin(xr_pose, tcp_pose)
        self.previous_target_tcp = tcp_pose.copy()
        self.previous_target_q = self.arm.get_current_joints()
        self.enabled = True
        print("Teleop enabled.")

    def _run_enabled_cycle(self, xr_pose):
        raw_target_tcp = self.mapper.target_tcp_pose(xr_pose)
        target_tcp = self.safety.limit_tcp_step(raw_target_tcp, self.previous_target_tcp)
        self.previous_target_tcp = target_tcp.copy()

        if self.config.arm.control_mode == "servo_l":
            self.arm.servo_linear(target_tcp)
            return target_tcp, None

        q_near = self.previous_target_q
        if q_near is None:
            q_near = self.arm.get_current_joints()
        target_q = self.arm.inverse_kinematics(target_tcp, q_near=q_near)
        if target_q is None:
            self._warn_throttled("IK failed or RTDE getInverseKinematics is unavailable.")
            return target_tcp, None

        safe_q = self.safety.limit_joint_step(target_q, self.previous_target_q)
        self.previous_target_q = safe_q.copy()
        self.arm.servo_joints(safe_q)
        return target_tcp, safe_q

    def _run_return_home_cycle(self, gripper_raw):
        current_q = self.arm.get_current_joints()
        previous_q = self.previous_target_q if self.previous_target_q is not None else current_q
        safe_q = self._limit_return_home_joint_step(self.return_home_target_q, previous_q)
        self.previous_target_q = safe_q.copy()

        target_tcp = self.arm.forward_kinematics(safe_q)
        if target_tcp is None:
            target_tcp = self.arm.get_current_tcp_pose()

        self.arm.servo_joints(safe_q)
        self._write_state("return_home", gripper_raw, target_tcp, safe_q)

        now = time.monotonic()
        joint_error = float(np.max(np.abs(self.return_home_target_q - current_q)))
        if joint_error <= self.config.return_home.tolerance:
            if self.return_home_reached_since is None:
                self.return_home_reached_since = now
            if now - self.return_home_reached_since >= self.config.return_home.hold_sec:
                self._finish_return_home("reached")
            return

        self.return_home_reached_since = None
        elapsed = now - self.return_home_started_at
        if elapsed >= self.config.return_home.timeout_sec:
            self._finish_return_home(f"timed out at joint_error={joint_error:.4f}rad")

    def _limit_return_home_joint_step(self, target_q, previous_q):
        target = np.array(target_q, dtype=np.float64)
        previous = np.array(previous_q, dtype=np.float64)
        delta = target - previous
        max_step = float(self.config.return_home.max_joint_step)
        if max_step > 0.0:
            delta = np.clip(delta, -max_step, max_step)
        return previous + delta

    def _finish_return_home(self, reason: str):
        self.arm.stop_servo()
        self.return_home_active = False
        self.return_home_started_at = None
        self.return_home_reached_since = None
        self.previous_target_tcp = None
        self.previous_target_q = None
        self.mapper.reset()
        self._motion_locked_until_grip_release = True
        print(f"Return-home {reason}; requesting recorder stop.")
        if self.recorder_control is not None:
            self.recorder_control.request("stop")

    def run(self):
        if self.gripper is not None:
            self.gripper.start()

        print(
            "Teleop started: "
            f"robot={self.config.arm.robot_ip}, "
            f"mode={self.config.arm.control_mode}, "
            f"rate={self.config.arm.rate_hz}Hz, "
            f"gripper={self.config.gripper.port if self.gripper else 'disabled'}"
        )
        print("Hold right grip to move. Press right stick click for emergency stop.")
        print(
            "Grip enable: "
            f"start>={self.config.xr.grip_threshold:.2f}, "
            f"release<{self.config.xr.grip_release_threshold:.2f}"
        )
        if self.recorder_control is not None:
            print(
                "Recorder buttons: "
                f"start={self.config.recorder.start_button}, "
                f"stop={self.config.recorder.stop_button}"
                f"{' -> return home then stop' if self.config.return_home.enabled else ''}, "
                f"save={self.config.recorder.save_button}, "
                f"discard={self.config.recorder.discard_button}"
            )

        while not self.stop_requested:
            gripper_raw = self._handle_gripper()

            if self.xr.get_button(self.config.xr.emergency_button):
                print("Emergency button pressed.")
                break

            if self.return_home_active:
                self._handle_recorder_control()
                try:
                    self._run_return_home_cycle(gripper_raw)
                except Exception as exc:
                    self._warn_throttled(f"Return-home cycle failed: {exc}")
                    self.arm.stop_servo()
                    time.sleep(0.02)
                continue

            if not self._xr_is_fresh():
                self._warn_throttled("XR input timed out; holding robot.")
                self._deactivate()
                self._write_state("xr_timeout", gripper_raw)
                time.sleep(0.01)
                continue

            self._handle_recorder_control()
            if self.return_home_active:
                continue

            xr_pose = self.xr.get_pose(self.config.xr.pose_source)
            if not np.all(np.isfinite(np.array(xr_pose, dtype=float))):
                self._warn_throttled("XR pose contains non-finite values; holding robot.")
                self._deactivate()
                self._write_state("invalid_xr_pose", gripper_raw)
                time.sleep(0.01)
                continue

            grip = self.xr.get_value(self.config.xr.grip_name)
            active = self._motion_enabled_by_grip(grip)

            if not active:
                self._motion_locked_until_grip_release = False
                self._deactivate()
                self._write_state("idle", gripper_raw)
                time.sleep(0.01)
                continue

            if self._motion_locked_until_grip_release:
                self._deactivate()
                self._write_state("idle", gripper_raw)
                time.sleep(0.01)
                continue

            if not self.enabled:
                self._activate(xr_pose)

            try:
                target_tcp, target_q = self._run_enabled_cycle(xr_pose)
            except Exception as exc:
                self._warn_throttled(f"Control cycle failed: {exc}")
                self.arm.stop_servo()
                time.sleep(0.02)
                continue

            self._write_state("running", gripper_raw, target_tcp, target_q)

        self.shutdown()

    def shutdown(self):
        print("Shutting down teleop.")
        self._deactivate()
        if self.gripper is not None:
            self.gripper.stop()
        self.arm.close()
        if self.recorder_control is not None:
            self.recorder_control.close()
        self.xr.close()


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = config_from_args(args)
    runner = TeleopRunner(config)
    signal.signal(signal.SIGINT, runner.request_stop)
    signal.signal(signal.SIGTERM, runner.request_stop)
    runner.run()


if __name__ == "__main__":
    main()
