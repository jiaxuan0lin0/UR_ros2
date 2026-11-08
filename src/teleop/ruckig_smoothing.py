import time

import numpy as np


class RuckigPositionSmoother:
    """Jerk-limited online smoothing for Cartesian target positions."""

    def __init__(
        self,
        cycle_time: float,
        max_velocity: float,
        max_acceleration: float,
        max_jerk: float,
        prediction_sec: float = 0.0,
        velocity_hold_sec: float = 0.015,
    ):
        try:
            import ruckig
        except ImportError as exc:
            raise RuntimeError(
                "Ruckig target smoothing requires the Python package 'ruckig'. "
                "Install it in the teleop Python environment with: "
                "python3 -m pip install ruckig"
            ) from exc

        self.ruckig = ruckig
        self.cycle_time = max(float(cycle_time), 1.0e-4)
        self.max_velocity = max(1.0e-6, float(max_velocity))
        self.max_acceleration = max(1.0e-6, float(max_acceleration))
        self.max_jerk = max(1.0e-6, float(max_jerk))
        self.prediction_sec = max(0.0, float(prediction_sec))
        self.velocity_hold_sec = max(0.0, float(velocity_hold_sec))
        self.otg = ruckig.Ruckig(3, self.cycle_time)
        self.input = ruckig.InputParameter(3)
        self.output = ruckig.OutputParameter(3)
        self.initialized = False
        self._last_raw_position = None
        self._last_raw_time = None
        self._raw_velocity = np.zeros(3, dtype=np.float64)

        self.input.max_velocity = [self.max_velocity] * 3
        self.input.max_acceleration = [self.max_acceleration] * 3
        self.input.max_jerk = [self.max_jerk] * 3

    def reset(self, position=None):
        self.initialized = False
        self._last_raw_position = None
        self._last_raw_time = None
        self._raw_velocity = np.zeros(3, dtype=np.float64)
        if position is not None:
            self.initialize(position)

    def initialize(self, position, now=None):
        values = self._valid_position(position)
        self.input.current_position = values.tolist()
        self.input.current_velocity = [0.0, 0.0, 0.0]
        self.input.current_acceleration = [0.0, 0.0, 0.0]
        self.input.target_position = values.tolist()
        self.input.target_velocity = [0.0, 0.0, 0.0]
        self.input.target_acceleration = [0.0, 0.0, 0.0]
        self.output = self.ruckig.OutputParameter(3)
        self.initialized = True
        self._last_raw_position = values.copy()
        self._last_raw_time = time.monotonic() if now is None else float(now)
        self._raw_velocity = np.zeros(3, dtype=np.float64)
        return values

    def update(self, target_position, now=None):
        target = self._valid_position(target_position)
        if now is None:
            now = time.monotonic()
        if not self.initialized:
            return self.initialize(target, now)

        self._update_raw_velocity(target, now)
        target_velocity = self._target_velocity()
        predicted_target = target + target_velocity * self.prediction_sec

        self.input.target_position = predicted_target.tolist()
        self.input.target_velocity = [0.0, 0.0, 0.0]
        self.input.target_acceleration = [0.0, 0.0, 0.0]
        self.input.max_velocity = [self.max_velocity] * 3
        self.input.max_acceleration = [self.max_acceleration] * 3
        self.input.max_jerk = [self.max_jerk] * 3

        try:
            result = self.otg.update(self.input, self.output)
        except Exception:
            return self.initialize(target, now)

        if result.value < 0:
            return self.initialize(target, now)

        position = np.array(self.output.new_position, dtype=np.float64)
        if not np.all(np.isfinite(position)):
            return self.initialize(target, now)

        self.output.pass_to_input(self.input)
        return position

    def _update_raw_velocity(self, target: np.ndarray, now: float):
        if self._last_raw_position is None or self._last_raw_time is None:
            self._last_raw_position = target.copy()
            self._last_raw_time = now
            self._raw_velocity = np.zeros(3, dtype=np.float64)
            return

        delta = target - self._last_raw_position
        if float(np.linalg.norm(delta)) <= 1.0e-7:
            if now - self._last_raw_time > self.velocity_hold_sec:
                self._raw_velocity = np.zeros(3, dtype=np.float64)
            return

        dt = max(1.0e-4, min(now - self._last_raw_time, 0.1))
        velocity = delta / dt
        speed = float(np.linalg.norm(velocity))
        if speed > self.max_velocity > 0.0:
            velocity *= self.max_velocity / speed
        self._raw_velocity = velocity
        self._last_raw_position = target.copy()
        self._last_raw_time = now

    def _target_velocity(self) -> np.ndarray:
        velocity = np.array(self._raw_velocity, dtype=np.float64)
        if not np.all(np.isfinite(velocity)):
            return np.zeros(3, dtype=np.float64)

        speed = float(np.linalg.norm(velocity))
        if speed <= 1.0e-5:
            return np.zeros(3, dtype=np.float64)
        if speed > self.max_velocity > 0.0:
            velocity *= self.max_velocity / speed
        return velocity

    @staticmethod
    def _valid_position(position) -> np.ndarray:
        values = np.array(position, dtype=np.float64)
        if values.size != 3 or not np.all(np.isfinite(values)):
            raise ValueError("RuckigPositionSmoother expected a finite xyz position")
        return values
