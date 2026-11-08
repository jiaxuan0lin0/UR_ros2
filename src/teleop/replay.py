import json
import time
from pathlib import Path

import numpy as np

from teleop.gripper_direct import DirectDHGripper
from teleop.motion_math import nearest_joint_equivalent


SAMPLED_REPLAY_FILE = Path("/tmp/ur_sampled_replay.json")


def _coerce_replay_gripper_raw(gripper_raw):
    if gripper_raw is None:
        return None
    try:
        raw = int(round(float(gripper_raw)))
    except (TypeError, ValueError):
        return None
    return max(
        DirectDHGripper.MIN_POSITION,
        min(raw, DirectDHGripper.MAX_POSITION),
    )


def _replay_joint_distance(current_q, replay_q) -> float:
    replay_q = np.array(replay_q, dtype=np.float64)
    current_q = nearest_joint_equivalent(current_q, replay_q)
    return float(np.linalg.norm(current_q - replay_q))


class ReplayMixin:
    """Episode replay for TeleopRunner; uses the state set up in TeleopRunner.__init__."""

    def _clear_replay_buffer(self, reason: str | None = None):
        self.replay_buffer = []
        self._last_replay_capture_time = None
        self._replay_capture_started_at = None
        self._replay_ready = False
        self.replay_pending_until = None
        self.replay_active = False
        self.replay_started_at = None
        self._replay_duration = 0.0
        self._replay_index = 0
        self._replay_gripper_index = 0
        if self.config.replay.enabled and reason:
            print(f"Replay buffer cleared: {reason}.")

    def _capture_replay_point(
        self,
        target_q,
        target_tcp,
        gripper_raw,
        *,
        force: bool = False,
        phase: str = "running",
    ):
        if (
            not self.config.replay.enabled
            or not self._replay_episode_armed
            or target_q is None
        ):
            return

        now = time.monotonic()
        period = 1.0 / max(float(self.config.replay.record_hz), 1.0)
        if (
            not force
            and self._last_replay_capture_time is not None
            and now - self._last_replay_capture_time < period
        ):
            return

        q = np.array(target_q, dtype=np.float64)
        if q.size != 6 or not np.all(np.isfinite(q)):
            return

        if not self.replay_buffer:
            self._replay_capture_started_at = now
            replay_time = 0.0
        else:
            q = nearest_joint_equivalent(q, self.replay_buffer[-1]["q"])
            replay_time = now - self._replay_capture_started_at

        tcp = None
        if target_tcp is not None:
            tcp_array = np.array(target_tcp, dtype=np.float64)
            if tcp_array.size == 6 and np.all(np.isfinite(tcp_array)):
                tcp = tcp_array

        self.replay_buffer.append(
            {
                "time": float(replay_time),
                "q": q.copy(),
                "tcp": None if tcp is None else tcp.copy(),
                "gripper_raw": _coerce_replay_gripper_raw(gripper_raw),
                "phase": phase,
            }
        )
        self._last_replay_capture_time = now

    def _capture_replay_gripper_only_point(self, gripper_raw):
        if (
            not self.config.replay.enabled
            or not self._replay_episode_armed
            or not self.replay_buffer
        ):
            return

        raw = _coerce_replay_gripper_raw(gripper_raw)
        if raw is None:
            return

        previous_raw = _coerce_replay_gripper_raw(
            self.replay_buffer[-1].get("gripper_raw")
        )
        deadband = max(1, int(self.config.gripper.position_deadband))
        if previous_raw is not None and abs(raw - previous_raw) < deadband:
            return

        try:
            current_q = self.arm.get_current_joints()
            current_tcp = self.arm.get_current_tcp_pose()
        except Exception:
            return
        self._update_actual_cache(q=current_q, tcp=current_tcp)
        self._capture_replay_point(current_q, current_tcp, raw)

    def _schedule_replay_after_return_home(self, reason: str):
        if not self.config.replay.enabled:
            return

        was_armed = self._replay_episode_armed
        self._replay_episode_armed = False
        if reason != "reached":
            self._clear_replay_buffer(f"return-home {reason}")
            return

        if not was_armed:
            self._clear_replay_buffer("return-home without recorder start")
            print("Replay skipped: B was pressed without an A-started recorder episode.")
            return

        if not self._load_sampled_replay_buffer():
            self._clear_replay_buffer("sampled replay unavailable")
            print("Replay skipped: sampled recorder trajectory is unavailable.")
            return

        self._replay_ready = True
        self.replay_pending_until = None
        self.replay_active = False
        self._replay_gripper_index = 0
        return_home_points = sum(
            point.get("phase") == "return_home"
            for point in self.replay_buffer
        )
        print(
            "Replay ready: "
            f"{len(self.replay_buffer)} points, "
            f"return_home={return_home_points}. Press "
            f"{self.config.replay.trigger_button} to start."
        )

    def _load_sampled_replay_buffer(self) -> bool:
        try:
            with open(SAMPLED_REPLAY_FILE, "r") as f:
                payload = json.load(f)
        except FileNotFoundError:
            print(f"Replay file is missing: {SAMPLED_REPLAY_FILE}")
            return False
        except (OSError, json.JSONDecodeError) as exc:
            print(f"Replay file is unreadable: {exc}")
            return False

        if isinstance(payload, dict):
            raw_points = payload.get("points")
            episode_dir = payload.get("episode_dir")
        else:
            raw_points = payload
            episode_dir = None

        if not isinstance(raw_points, list):
            print("Replay file does not contain a points list.")
            return False

        loaded_points = []
        previous_q = None
        previous_time = 0.0
        for raw_point in raw_points:
            if not isinstance(raw_point, dict):
                continue

            try:
                q = np.array(raw_point.get("q"), dtype=np.float64)
            except Exception:
                continue
            if q.size != 6 or not np.all(np.isfinite(q)):
                continue
            if previous_q is not None:
                q = nearest_joint_equivalent(q, previous_q)

            tcp = None
            try:
                raw_tcp = raw_point.get("tcp")
                if raw_tcp is not None:
                    tcp_array = np.array(raw_tcp, dtype=np.float64)
                    if tcp_array.size == 6 and np.all(np.isfinite(tcp_array)):
                        tcp = tcp_array
            except Exception:
                tcp = None

            try:
                replay_time = float(raw_point.get("time"))
            except (TypeError, ValueError):
                replay_time = previous_time
            if not np.isfinite(replay_time):
                replay_time = previous_time
            replay_time = max(previous_time, replay_time)

            gripper_raw = _coerce_replay_gripper_raw(
                raw_point.get("gripper_target_raw")
            )
            if gripper_raw is None:
                gripper_raw = _coerce_replay_gripper_raw(
                    raw_point.get("gripper_raw")
                )
            phase = raw_point.get("phase")
            if not isinstance(phase, str) or not phase:
                phase = "running"

            loaded_points.append(
                {
                    "time": float(replay_time),
                    "q": q.copy(),
                    "tcp": None if tcp is None else tcp.copy(),
                    "gripper_raw": gripper_raw,
                    "phase": phase,
                }
            )
            previous_q = q.copy()
            previous_time = float(replay_time)

        if len(loaded_points) < 2:
            print("Replay file does not contain enough sampled points.")
            return False

        self.replay_buffer = loaded_points
        self._last_replay_capture_time = None
        self._replay_capture_started_at = None
        self._replay_ready = False
        self.replay_pending_until = None
        self.replay_active = False
        self.replay_started_at = None
        self._replay_duration = 0.0
        self._replay_index = 0
        self._replay_gripper_index = 0
        source = f" from {episode_dir}" if isinstance(episode_dir, str) and episode_dir else ""
        print(
            f"Loaded {len(loaded_points)} sampled replay points{source}."
        )
        return True

    def _handle_replay_button(self) -> bool:
        if not self.config.replay.enabled:
            return False

        pressed = self._replay_button_pressed()
        was_pressed = self._replay_button_was_pressed
        self._replay_button_was_pressed = pressed
        if not pressed or was_pressed:
            return False

        if self.replay_active or self.replay_pending_until is not None:
            return True
        if not self._replay_ready:
            print("Replay ignored: no returned-home trajectory is ready.")
            return False
        if len(self.replay_buffer) < 2:
            self._clear_replay_buffer("not enough replay points")
            print("Replay skipped: not enough recorded trajectory points.")
            return False

        self._replay_ready = False
        delay_sec = max(0.0, float(self.config.replay.delay_sec))
        if delay_sec > 0.0:
            self.replay_pending_until = time.monotonic() + delay_sec
            self.replay_active = False
            self._replay_gripper_index = 0
            self._set_replay_gripper_target(self._first_replay_gripper_raw())
            print(f"Replay button accepted; starting after {delay_sec:.2f}s.")
        else:
            self._start_replay()
        return True

    def _replay_button_pressed(self) -> bool:
        pressed_any = False
        for button in self._replay_button_names():
            try:
                pressed_any = pressed_any or bool(self.xr.get_button(button))
                continue
            except Exception:
                pass

            try:
                value = float(self.xr.get_value(button))
                threshold = min(
                    1.0,
                    max(0.0, float(self.config.replay.trigger_threshold)),
                )
                pressed_any = pressed_any or value >= threshold
            except Exception as exc:
                self._warn_throttled(
                    f"Replay trigger {button!r} is unavailable: {exc}",
                    interval_sec=2.0,
                )
        return pressed_any

    def _replay_button_names(self) -> list[str]:
        names = [
            name.strip()
            for name in str(self.config.replay.trigger_button).split(",")
            if name.strip()
        ]
        return names or ["left_grip"]

    def _run_replay_if_needed(self, gripper_raw) -> bool:
        if self.replay_pending_until is None and not self.replay_active:
            return False

        now = time.monotonic()
        if self.replay_pending_until is not None:
            if now < self.replay_pending_until:
                first_gripper_raw = self._first_replay_gripper_raw()
                if first_gripper_raw is not None:
                    gripper_raw = first_gripper_raw
                    self._set_replay_gripper_target(first_gripper_raw)
                self._hold_speed_backend_alive()
                self._write_state("replay_wait", gripper_raw)
                time.sleep(0.01)
                return True
            self._start_replay()

        if not self.replay_active:
            return False

        target_q, target_tcp, qdot_ff, replay_gripper_raw = self._replay_target(now)
        current_q = self.arm.get_current_joints()
        self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
        replay_gripper_raw = self._replay_gripper_for_current_q(
            current_q,
            replay_gripper_raw,
        )
        target_q = nearest_joint_equivalent(target_q, current_q)
        joint_error = target_q - current_q

        if replay_gripper_raw is not None and self.gripper is not None:
            gripper_raw = replay_gripper_raw
            self.gripper.set_target(int(replay_gripper_raw))

        if self.config.arm.control_mode == "speed_j_dls":
            qdot = qdot_ff + joint_error * float(self.config.replay.joint_gain)
            qdot = self._limit_joint_velocity(qdot)
            self._active_motion_kind = "speed"
            self.arm.speed_joints(qdot, self.config.arm.max_joint_acceleration)
            self._previous_joint_velocity = qdot.copy()
        else:
            previous_q = self.previous_target_q if self.previous_target_q is not None else current_q
            safe_q = self.safety.limit_joint_step(target_q, previous_q)
            target_q = safe_q.copy()
            self._active_motion_kind = "servo"
            self.arm.servo_joints(safe_q)

        self.previous_target_q = target_q.copy()
        if target_tcp is None:
            target_tcp = self._return_home_target_tcp(target_q)
        self.previous_target_tcp = target_tcp.copy()
        self._write_state("replay", gripper_raw, target_tcp, target_q)

        elapsed = (now - self.replay_started_at) * max(
            float(self.config.replay.speed_scale),
            1.0e-6,
        )
        if elapsed >= self._replay_duration:
            tolerance = max(0.0, float(self.config.replay.tolerance))
            if float(np.max(np.abs(joint_error))) <= tolerance:
                self._finish_replay("finished")
        return True

    def _start_replay(self):
        if len(self.replay_buffer) < 2:
            self.replay_pending_until = None
            return
        self.replay_pending_until = None
        self.replay_active = True
        self.replay_started_at = time.monotonic()
        self._replay_duration = max(0.0, float(self.replay_buffer[-1]["time"]))
        self._replay_index = 0
        self._replay_gripper_index = 0
        self._set_replay_gripper_target(self._first_replay_gripper_raw())
        current_q = self.arm.get_current_joints()
        current_tcp = self.arm.get_current_tcp_pose()
        self._update_actual_cache(q=current_q, tcp=current_tcp)
        self.previous_target_q = current_q
        self.previous_target_tcp = current_tcp
        self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
        self._last_speed_cycle_time = time.monotonic()
        print(f"Replay started: duration={self._replay_duration:.2f}s.")

    def _replay_target(self, now: float):
        speed_scale = max(float(self.config.replay.speed_scale), 1.0e-6)
        elapsed = (now - self.replay_started_at) * speed_scale
        elapsed = max(0.0, min(elapsed, self._replay_duration))

        while (
            self._replay_index + 1 < len(self.replay_buffer)
            and self.replay_buffer[self._replay_index + 1]["time"] < elapsed
        ):
            self._replay_index += 1

        current = self.replay_buffer[self._replay_index]
        if self._replay_index + 1 >= len(self.replay_buffer):
            return (
                current["q"].copy(),
                None if current["tcp"] is None else current["tcp"].copy(),
                np.zeros(6, dtype=np.float64),
                current["gripper_raw"],
            )

        nxt = self.replay_buffer[self._replay_index + 1]
        dt = max(float(nxt["time"] - current["time"]), 1.0e-6)
        alpha = min(1.0, max(0.0, (elapsed - current["time"]) / dt))
        q = current["q"] + (nxt["q"] - current["q"]) * alpha
        qdot_ff = (nxt["q"] - current["q"]) / dt * speed_scale

        tcp = None
        if current["tcp"] is not None and nxt["tcp"] is not None:
            tcp = current["tcp"] + (nxt["tcp"] - current["tcp"]) * alpha
        elif current["tcp"] is not None:
            tcp = current["tcp"].copy()

        gripper_raw = current["gripper_raw"] if alpha < 0.5 else nxt["gripper_raw"]
        return q, tcp, qdot_ff, gripper_raw

    def _first_replay_gripper_raw(self):
        if not self.replay_buffer:
            return None
        return _coerce_replay_gripper_raw(
            self.replay_buffer[0].get("gripper_raw")
        )

    def _set_replay_gripper_target(self, gripper_raw):
        raw = _coerce_replay_gripper_raw(gripper_raw)
        if raw is None or self.gripper is None:
            return raw
        self.gripper.set_target(raw)
        return raw

    def _replay_gripper_for_current_q(self, current_q, fallback_raw):
        fallback_raw = _coerce_replay_gripper_raw(fallback_raw)
        if not self.replay_buffer:
            return fallback_raw

        try:
            q = np.array(current_q, dtype=np.float64)
        except (TypeError, ValueError):
            return fallback_raw
        if q.size != 6 or not np.all(np.isfinite(q)):
            return fallback_raw

        last_index = len(self.replay_buffer) - 1
        index = min(max(int(self._replay_gripper_index), 0), last_index)
        max_search_index = min(last_index, max(self._replay_index + 1, index))

        while index < max_search_index:
            current_distance = _replay_joint_distance(
                q,
                self.replay_buffer[index]["q"],
            )
            next_distance = _replay_joint_distance(
                q,
                self.replay_buffer[index + 1]["q"],
            )
            if next_distance > current_distance:
                break
            index += 1

        self._replay_gripper_index = index
        gripper_raw = _coerce_replay_gripper_raw(
            self.replay_buffer[index].get("gripper_raw")
        )
        return fallback_raw if gripper_raw is None else gripper_raw

    def _finish_replay(self, reason: str):
        self._stop_active_motion()
        self.replay_active = False
        self.replay_pending_until = None
        self.replay_started_at = None
        self._replay_duration = 0.0
        self._replay_gripper_index = 0
        self.previous_target_q = None
        self.previous_target_tcp = None
        self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
        self._last_speed_cycle_time = None
        self.mapper.reset()
        self._motion_locked_until_grip_release = True
        self._grip_active = False
        print(f"Replay {reason}. Release grip before teleop resumes.")
