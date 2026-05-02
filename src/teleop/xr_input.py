import time


class XrInput:
    def __init__(self):
        try:
            import xrobotoolkit_sdk as xrt
        except ImportError as exc:
            raise RuntimeError(
                "xrobotoolkit_sdk is not importable. Start XRoboToolkit PC Service "
                "and run this script in the environment where the SDK is installed."
            ) from exc

        self._xrt = xrt
        self._xrt.init()

    def get_pose(self, name: str):
        if name == "left_controller":
            return self._xrt.get_left_controller_pose()
        if name == "right_controller":
            return self._xrt.get_right_controller_pose()
        if name == "headset":
            return self._xrt.get_headset_pose()
        raise ValueError(f"unsupported XR pose source: {name}")

    def get_value(self, name: str) -> float:
        if name == "left_trigger":
            return float(self._xrt.get_left_trigger())
        if name == "right_trigger":
            return float(self._xrt.get_right_trigger())
        if name == "left_grip":
            return float(self._xrt.get_left_grip())
        if name == "right_grip":
            return float(self._xrt.get_right_grip())
        raise ValueError(f"unsupported XR value: {name}")

    def get_button(self, name: str) -> bool:
        if name == "A":
            return bool(self._xrt.get_A_button())
        if name == "B":
            return bool(self._xrt.get_B_button())
        if name == "X":
            return bool(self._xrt.get_X_button())
        if name == "Y":
            return bool(self._xrt.get_Y_button())
        if name == "left_axis_click":
            return bool(self._xrt.get_left_axis_click())
        if name == "right_axis_click":
            return bool(self._xrt.get_right_axis_click())
        if name == "left_menu_button":
            return bool(self._xrt.get_left_menu_button())
        if name == "right_menu_button":
            return bool(self._xrt.get_right_menu_button())
        raise ValueError(f"unsupported XR button: {name}")

    def timestamp_sec(self) -> float:
        try:
            return float(self._xrt.get_time_stamp_ns()) * 1.0e-9
        except Exception:
            return time.monotonic()

    def close(self):
        self._xrt.close()

