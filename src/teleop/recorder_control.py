import shutil
import subprocess
import time


class RecorderControl:
    SERVICES = {
        "start": "/recorder/start",
        "stop": "/recorder/stop",
        "save": "/recorder/save",
        "discard": "/recorder/discard",
    }

    def __init__(self, button_map: dict[str, str], debounce_sec: float = 0.5):
        self.button_map = dict(button_map)
        self.debounce_sec = float(debounce_sec)
        self.previous_button_state = {
            button: False for button in set(self.button_map.values())
        }
        self.last_call_time = {action: 0.0 for action in self.SERVICES}
        self.pending_calls = []
        self.last_missing_ros2_log_time = 0.0

    def poll(self, get_button):
        now = time.monotonic()
        for action, button in self.button_map.items():
            pressed = bool(get_button(button))
            was_pressed = self.previous_button_state.get(button, False)
            self.previous_button_state[button] = pressed
            if not pressed or was_pressed:
                continue
            if now - self.last_call_time[action] < self.debounce_sec:
                continue
            self.last_call_time[action] = now
            self._call(action, now)

        self._drain_completed()

    def request(self, action: str):
        if action not in self.SERVICES:
            raise ValueError(f"unsupported recorder action: {action}")
        now = time.monotonic()
        if now - self.last_call_time[action] < self.debounce_sec:
            return
        self.last_call_time[action] = now
        self._call(action, now)

    def request_blocking(self, action: str, timeout_sec: float = 3.0) -> bool:
        if action not in self.SERVICES:
            raise ValueError(f"unsupported recorder action: {action}")

        now = time.monotonic()
        if now - self.last_call_time[action] < self.debounce_sec:
            return False
        self.last_call_time[action] = now
        return self._call_blocking(action, now, timeout_sec=timeout_sec)

    def _call(self, action: str, now: float):
        if shutil.which("ros2") is None:
            if now - self.last_missing_ros2_log_time >= 2.0:
                print("ros2 command is not available. Source ROS 2 before starting teleop.")
                self.last_missing_ros2_log_time = now
            return

        service_name = self.SERVICES[action]
        cmd = [
            "timeout",
            "2s",
            "ros2",
            "service",
            "call",
            service_name,
            "std_srvs/srv/Trigger",
            "{}",
        ]
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.pending_calls.append((action, service_name, process))
        print(f"Recorder {action} requested.")

    def _call_blocking(self, action: str, now: float, timeout_sec: float) -> bool:
        if shutil.which("ros2") is None:
            if now - self.last_missing_ros2_log_time >= 2.0:
                print("ros2 command is not available. Source ROS 2 before starting teleop.")
                self.last_missing_ros2_log_time = now
            return False

        service_name = self.SERVICES[action]
        cmd = [
            "ros2",
            "service",
            "call",
            service_name,
            "std_srvs/srv/Trigger",
            "{}",
        ]
        print(f"Recorder {action} requested.")
        try:
            process = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=max(0.1, float(timeout_sec)),
            )
        except subprocess.TimeoutExpired:
            print(f"Recorder {action} timed out waiting for {service_name}.")
            return False

        output = (process.stdout + process.stderr).strip()
        message = self._last_nonempty_line(output)
        if process.returncode == 0:
            if message:
                print(f"Recorder {action} response: {message}")
            else:
                print(f"Recorder {action} command completed.")
            return True

        print(f"Recorder {action} failed: {message or 'ros2 service call failed'}")
        return False

    def _drain_completed(self):
        still_pending = []
        for action, service_name, process in self.pending_calls:
            if process.poll() is None:
                still_pending.append((action, service_name, process))
                continue

            stdout, stderr = process.communicate()
            output = (stdout + stderr).strip()
            if process.returncode == 0:
                message = self._last_nonempty_line(output)
                if message:
                    print(f"Recorder {action} response: {message}")
                else:
                    print(f"Recorder {action} command completed.")
            elif process.returncode == 124:
                print(f"Recorder {action} timed out waiting for {service_name}.")
            else:
                message = self._last_nonempty_line(output)
                print(f"Recorder {action} failed: {message or 'ros2 service call failed'}")
        self.pending_calls = still_pending

    @staticmethod
    def _last_nonempty_line(text: str):
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        return lines[-1] if lines else None

    def close(self):
        deadline = time.monotonic() + 1.0
        while self.pending_calls and time.monotonic() < deadline:
            self._drain_completed()
            time.sleep(0.01)

        for _action, _service_name, process in self.pending_calls:
            if process.poll() is None:
                process.terminate()
        self.pending_calls = []
