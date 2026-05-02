import sys
import threading
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CONTROLLERS_SRC = REPO_ROOT / "src" / "controllers"
if CONTROLLERS_SRC.exists() and str(CONTROLLERS_SRC) not in sys.path:
    sys.path.insert(0, str(CONTROLLERS_SRC))


class DirectDHGripper:
    MIN_FORCE = 20
    MAX_FORCE = 100
    MIN_SPEED = 1
    MAX_SPEED = 100
    MIN_POSITION = 0
    MAX_POSITION = 1000

    def __init__(
        self,
        port: str,
        baudrate: int,
        rate_hz: float,
        speed: int,
        force: int,
        position_deadband: int,
        init_timeout_sec: float,
        read_state_hz: float,
    ):
        from controllers.dh_gripper import dh_modbus_gripper

        self.port = port
        self.baudrate = int(baudrate)
        self.period_sec = 1.0 / max(float(rate_hz), 1.0)
        self.speed = self._clamp(speed, self.MIN_SPEED, self.MAX_SPEED)
        self.force = self._clamp(force, self.MIN_FORCE, self.MAX_FORCE)
        self.position_deadband = max(0, int(position_deadband))
        self.init_timeout_sec = float(init_timeout_sec)
        self.read_state_period_sec = 1.0 / max(float(read_state_hz), 0.1)
        self._gripper = dh_modbus_gripper.dh_modbus_gripper()
        self._target_lock = threading.Lock()
        self._target_raw = self.MIN_POSITION
        self._current_raw = None
        self._last_sent_raw = None
        self._last_read_time = 0.0
        self._stop_event = threading.Event()
        self._thread = None
        self._ready = False

    @staticmethod
    def _clamp(value, min_value, max_value) -> int:
        return max(min_value, min(int(round(value)), max_value))

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="dh_gripper", daemon=True)
        self._thread.start()

    def set_target(self, raw_position: int):
        target = self._clamp(raw_position, self.MIN_POSITION, self.MAX_POSITION)
        with self._target_lock:
            self._target_raw = target

    def get_target(self) -> int:
        with self._target_lock:
            return int(self._target_raw)

    def get_current(self) -> int | None:
        with self._target_lock:
            return None if self._current_raw is None else int(self._current_raw)

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._gripper.close()
        except Exception:
            pass

    def _connect_and_initialize(self):
        ret = self._gripper.open(self.port, self.baudrate)
        if ret < 0:
            raise RuntimeError(f"failed to open DH gripper port: {self.port}")

        self._gripper.Initialization()
        deadline = time.monotonic() + self.init_timeout_sec
        while time.monotonic() < deadline and not self._stop_event.is_set():
            init_state = self._gripper.GetInitState()
            if init_state == 1:
                self._gripper.SetTargetForce(self.force)
                self._gripper.SetTargetSpeed(self.speed)
                self._ready = True
                return
            time.sleep(0.2)
        raise RuntimeError("DH gripper initialization timed out.")

    def _run(self):
        try:
            self._connect_and_initialize()
            print(f"DH gripper ready on {self.port}.")
        except Exception as exc:
            print(f"DH gripper disabled: {exc}")
            return

        while not self._stop_event.is_set():
            target = self.get_target()
            if (
                self._last_sent_raw is None
                or abs(target - self._last_sent_raw) >= self.position_deadband
            ):
                try:
                    self._gripper.SetTargetPosition(target)
                    self._last_sent_raw = target
                except Exception as exc:
                    print(f"DH gripper command failed: {exc}")
                    self._ready = False
                    return

            now = time.monotonic()
            if now - self._last_read_time >= self.read_state_period_sec:
                self._last_read_time = now
                try:
                    current = self._gripper.GetCurrentPosition()
                    if current is not None:
                        with self._target_lock:
                            self._current_raw = self._clamp(
                                current,
                                self.MIN_POSITION,
                                self.MAX_POSITION,
                            )
                except Exception:
                    pass
            time.sleep(self.period_sec)
