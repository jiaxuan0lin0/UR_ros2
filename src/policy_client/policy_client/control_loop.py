"""Trigger service handlers and the background policy control loop."""

from __future__ import annotations

import threading
import time

import rclpy


class ControlLoopMixin:
    """Services and loop thread; state lives in PolicyControlNode.__init__."""

    def _start_cb(self, _request, response):
        ok, message = self._start_control()
        response.success = ok
        response.message = message
        return response

    def _stop_cb(self, _request, response):
        ok, message = self._stop_control()
        response.success = ok
        response.message = message
        return response

    def _step_cb(self, _request, response):
        if self._is_running():
            response.success = False
            response.message = 'policy control loop is already running'
            return response

        try:
            ok, message = self._check_controller_ready()
            if not ok:
                response.success = False
                response.message = message
                return response
            self._ensure_connected()
            if self.reset_policy_on_start:
                self._reset_policy_history()
            ok, message = self._execute_step()
        except Exception as exc:
            ok = False
            message = str(exc)
        response.success = ok
        response.message = message
        return response

    def _reset_policy_cb(self, _request, response):
        try:
            self._reset_policy_history()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = 'policy history reset'
        return response

    def _start_control(self) -> tuple[bool, str]:
        instruction = self.instruction.strip()
        if not instruction:
            return False, 'instruction parameter must be non-empty'

        with self._state_lock:
            if self._running:
                return False, 'policy control loop is already running'
            self._running = True
            self._stop_event.clear()
            self._pending_executed_actions = (
                [[0.0] * 7] if self.send_initial_zero_action else None
            )
        self._clear_async_policy_request(increment_generation=True)

        try:
            ok, message = self._check_controller_ready()
            if not ok:
                raise RuntimeError(message)
            self._ensure_connected()
            if self.reset_policy_on_start:
                self._reset_policy_history()
            self.rtc_control.reset()
        except Exception as exc:
            with self._state_lock:
                self._running = False
            return False, f'failed to start policy control: {exc}'

        self._run_thread = threading.Thread(
            target=self._run_loop,
            name='policy_control_loop',
            daemon=True,
        )
        self._run_thread.start()
        return True, 'policy control loop started'

    def _stop_control(self) -> tuple[bool, str]:
        with self._state_lock:
            was_running = self._running
            self._running = False
            self._stop_event.set()

        if self._run_thread is not None and self._run_thread.is_alive():
            self._run_thread.join(timeout=2.0)
        self._clear_async_policy_request(increment_generation=True)
        self._stop_controller_motion()
        if not was_running:
            return False, 'policy control loop was not running'
        return True, 'policy control loop stopped'

    def _stop_controller_motion(self) -> None:
        stop = getattr(self.controller, 'stop', None)
        if not callable(stop):
            return
        try:
            stop()
        except Exception as exc:
            self.get_logger().warn(f'controller stop failed: {exc}')

    def _run_loop(self) -> None:
        period_sec = 1.0 / max(self.rate_hz, 0.01)
        try:
            while not self._stop_event.is_set() and rclpy.ok():
                started = time.monotonic()
                try:
                    ok, message = self._execute_step()
                except Exception as exc:
                    ok = False
                    message = str(exc)

                if not ok:
                    self.get_logger().error(f'policy control step failed: {message}')
                    if self.stop_on_error:
                        break
                else:
                    self._log_step(message)

                remaining = period_sec - (time.monotonic() - started)
                if remaining > 0.0:
                    self._stop_event.wait(timeout=remaining)
        finally:
            with self._state_lock:
                self._running = False

    def _ensure_connected(self) -> None:
        with self._state_lock:
            if self._connected:
                return
        health = self.policy.health()
        self.get_logger().info(f'policy health: {health}')
        self.observation_source.connect()
        with self._state_lock:
            self._connected = True

    def _check_controller_ready(self) -> tuple[bool, str]:
        if self.dry_run:
            return True, 'dry-run does not require controller action server'
        if not self.controller.wait_ready():
            return (
                False,
                'controller action server not ready: '
                f'{self.controller_action_name}',
            )
        return True, 'controller action server ready'

    def _is_running(self) -> bool:
        with self._state_lock:
            return self._running

    def _log_step(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_log_time < 1.0:
            return
        self._last_log_time = now
        self.get_logger().info(message)
