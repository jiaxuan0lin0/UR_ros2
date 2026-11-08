import time

import numpy as np

from teleop.motion_math import nearest_joint_equivalent


class ReturnHomeMixin:
    """Return-home motion for TeleopRunner; uses the state set up in TeleopRunner.__init__."""

    def _start_return_home(self):
        if self.return_home_active:
            print("Return-home is already running.")
            return

        if self.enabled:
            self._deactivate()

        current_q = self.arm.get_current_joints()
        current_tcp = self.arm.get_current_tcp_pose()
        self._update_actual_cache(q=current_q, tcp=current_tcp)
        self.previous_target_q = current_q
        self.previous_target_tcp = current_tcp
        self.return_home_active = True
        self.return_home_started_at = time.monotonic()
        self.return_home_reached_since = None
        self._force_next_state_write = True
        gripper_raw = self._current_gripper_target_raw()
        targets = self._write_return_home_state(
            gripper_raw,
            current_q=current_q,
            force=True,
        )
        if targets is not None:
            target_q, target_tcp = targets
            self._capture_replay_point(
                target_q,
                target_tcp,
                gripper_raw,
                force=True,
                phase="return_home",
            )
        print("Return-home started; recorder will stop after home is reached.")

    def _return_home_record_target_q(self, current_q, final: bool = False):
        current = np.array(current_q, dtype=np.float64)
        if final:
            return nearest_joint_equivalent(self.return_home_target_q, current)

        if self.config.arm.control_mode == "speed_j_dls":
            if self._return_home_joint_error(current) <= self.config.return_home.tolerance:
                return nearest_joint_equivalent(self.return_home_target_q, current)
            qdot = self._return_home_joint_velocity(current)
            return self._return_home_speed_target_q(current, qdot)

        previous_q = self.previous_target_q if self.previous_target_q is not None else current
        return self._limit_return_home_joint_step(self.return_home_target_q, previous_q)

    def _write_return_home_state(
        self,
        gripper_raw=None,
        current_q=None,
        final: bool = False,
        force: bool = False,
    ):
        try:
            if current_q is None:
                current_q = self.arm.get_current_joints()
            current_q = np.array(current_q, dtype=np.float64)
            self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
            target_q = self._return_home_record_target_q(current_q, final=final)
            target_tcp = self._return_home_target_tcp(target_q)
            if gripper_raw is None:
                gripper_raw = self._current_gripper_target_raw()
            self._write_state("return_home", gripper_raw, target_tcp, target_q, force=force)
            return target_q, target_tcp
        except Exception as exc:
            self._warn_throttled(f"Failed to write return-home recorder state: {exc}")
            return None

    def _run_return_home_cycle(self, gripper_raw):
        if self.config.arm.control_mode == "speed_j_dls":
            return self._run_return_home_speed_cycle(gripper_raw)
        return self._run_return_home_servo_cycle(gripper_raw)

    def _run_return_home_speed_cycle(self, gripper_raw):
        current_q = self.arm.get_current_joints()
        self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
        target_q = nearest_joint_equivalent(self.return_home_target_q, current_q)
        if self._return_home_joint_error(current_q) <= self.config.return_home.tolerance:
            self.previous_target_q = target_q.copy()
            self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
            self._active_motion_kind = "speed"
            target_tcp = self._return_home_target_tcp(target_q)
            self._write_state(
                "return_home",
                gripper_raw,
                target_tcp,
                target_q,
            )
            self._capture_replay_point(
                target_q,
                target_tcp,
                gripper_raw,
                phase="return_home",
            )
            self._update_return_home_progress(current_q)
            return

        qdot = self._return_home_joint_velocity(current_q)
        target_q = self._return_home_speed_target_q(current_q, qdot)
        self.previous_target_q = target_q.copy()
        self._previous_joint_velocity = qdot.copy()

        self._active_motion_kind = "speed"
        self.arm.speed_joints(qdot, self.config.arm.max_joint_acceleration)
        target_tcp = self._return_home_target_tcp(target_q)
        self._write_state("return_home", gripper_raw, target_tcp, target_q)
        self._capture_replay_point(
            target_q,
            target_tcp,
            gripper_raw,
            phase="return_home",
        )
        self._update_return_home_progress(current_q)

    def _run_return_home_servo_cycle(self, gripper_raw):
        current_q = self.arm.get_current_joints()
        self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
        previous_q = self.previous_target_q if self.previous_target_q is not None else current_q
        safe_q = self._limit_return_home_joint_step(self.return_home_target_q, previous_q)
        self.previous_target_q = safe_q.copy()

        self._active_motion_kind = "servo"
        self.arm.servo_joints(safe_q)
        target_tcp = self._return_home_target_tcp(safe_q)
        self._write_state("return_home", gripper_raw, target_tcp, safe_q)
        self._capture_replay_point(
            safe_q,
            target_tcp,
            gripper_raw,
            phase="return_home",
        )
        self._update_return_home_progress(current_q)

    def _update_return_home_progress(self, current_q):
        now = time.monotonic()
        joint_error = self._return_home_joint_error(current_q)
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

    def _return_home_joint_error(self, current_q) -> float:
        current = np.array(current_q, dtype=np.float64)
        target = nearest_joint_equivalent(self.return_home_target_q, current)
        return float(np.max(np.abs(target - current)))

    def _return_home_joint_velocity(self, current_q):
        current = np.array(current_q, dtype=np.float64)
        target = nearest_joint_equivalent(self.return_home_target_q, current)
        error = target - current
        abs_error = np.abs(error)

        max_speed = abs(float(self.config.arm.max_joint_speed))
        max_step = abs(float(self.config.return_home.max_joint_step))
        if max_step > 0.0:
            step_speed = max_step / max(self.arm.servo_time, 1.0e-6)
            max_speed = step_speed if max_speed <= 0.0 else min(max_speed, step_speed)

        acceleration = abs(float(self.config.arm.max_joint_acceleration))
        tolerance = max(0.0, float(self.config.return_home.tolerance))
        if acceleration > 0.0:
            braking_speed = np.sqrt(2.0 * acceleration * abs_error) * 0.5
        else:
            braking_speed = np.full(6, max_speed, dtype=np.float64)

        crawl_gain = 2.5
        crawl_speed = crawl_gain * abs_error
        desired_speed = np.minimum(braking_speed, crawl_speed)
        if max_speed > 0.0:
            desired_speed = np.minimum(desired_speed, max_speed)
        desired = np.sign(error) * desired_speed
        desired[abs_error <= tolerance] = 0.0
        limited = self._limit_joint_velocity(desired)

        max_no_overshoot_speed = abs_error / max(self.arm.servo_time, 1.0e-6)
        limited = np.sign(error) * np.minimum(np.abs(limited), max_no_overshoot_speed)
        limited[abs_error <= tolerance] = 0.0
        return limited

    def _return_home_speed_target_q(self, current_q, qdot):
        current = np.array(current_q, dtype=np.float64)
        target = nearest_joint_equivalent(self.return_home_target_q, current)
        next_q = current + np.array(qdot, dtype=np.float64) * self.arm.servo_time

        lower = np.minimum(current, target)
        upper = np.maximum(current, target)
        return np.minimum(np.maximum(next_q, lower), upper)

    def _return_home_target_tcp(self, target_q):
        if self.local_kinematics is not None:
            try:
                return self.local_kinematics.forward_pose(target_q, self.tcp_offset)
            except Exception as exc:
                self._warn_throttled(f"Local return-home FK failed: {exc}")

        target_tcp = self.arm.forward_kinematics(target_q)
        if target_tcp is not None:
            return target_tcp

        detail = self.arm.last_forward_kinematics_error
        if detail:
            self._warn_throttled(f"RTDE return-home FK failed; using current TCP: {detail}")
        return self.arm.get_current_tcp_pose()

    def _limit_return_home_joint_step(self, target_q, previous_q):
        target = np.array(target_q, dtype=np.float64)
        previous = np.array(previous_q, dtype=np.float64)
        delta = target - previous
        max_step = float(self.config.return_home.max_joint_step)
        if max_step > 0.0:
            delta = np.clip(delta, -max_step, max_step)
        return previous + delta

    def _finish_return_home(self, reason: str):
        self._stop_active_motion()
        gripper_raw = self._current_gripper_target_raw()
        targets = self._write_return_home_state(
            gripper_raw,
            final=(reason == "reached"),
            force=True,
        )
        if targets is not None:
            target_q, target_tcp = targets
            self._capture_replay_point(
                target_q,
                target_tcp,
                gripper_raw,
                force=True,
                phase="return_home",
            )
        stop_grace_sec = max(0.0, float(self.config.return_home.stop_grace_sec))
        if stop_grace_sec > 0.0:
            time.sleep(stop_grace_sec)
        self.return_home_active = False
        self.return_home_started_at = None
        self.return_home_reached_since = None
        self.previous_target_tcp = None
        self.previous_target_q = None
        self.mapper.reset()
        self._motion_locked_until_grip_release = False
        self._grip_active = False
        print(f"Return-home {reason}; requesting recorder stop.")
        recorder_stopped = True
        if self.recorder_control is not None:
            recorder_stopped = self.recorder_control.request_blocking(
                "stop",
                timeout_sec=3.0,
            )
        if recorder_stopped:
            self._schedule_replay_after_return_home(reason)
        else:
            self._clear_replay_buffer("recorder stop did not complete")
            print("Replay skipped: recorder stop did not complete.")
