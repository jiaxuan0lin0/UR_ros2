"""Keyboard client for controlling the recorder services."""

import select
import sys
import termios
import tty
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


KEY_TO_SERVICE = {
    "r": "/recorder/start",
    "c": "/recorder/stop",
    "s": "/recorder/save",
    "q": "/recorder/discard",
}


@contextmanager
def raw_terminal():
    if not sys.stdin.isatty():
        yield
        return

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        yield
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def read_key(timeout_sec: float = 0.1):
    if not sys.stdin.isatty():
        return None

    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not readable:
        return None

    return sys.stdin.read(1).lower()


class KeyboardControl(Node):
    def __init__(self):
        super().__init__("recorder_keyboard_control")
        self.service_clients = {
            key: self.create_client(Trigger, service_name)
            for key, service_name in KEY_TO_SERVICE.items()
        }

    def wait_for_services(self):
        for service_name, client in zip(KEY_TO_SERVICE.values(), self.service_clients.values()):
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {service_name}...")

    def call_service(self, key: str):
        client = self.service_clients[key]
        request = Trigger.Request()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error(f"Service call failed for key '{key}'")
            return

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)


def print_help():
    print(
        "\nRecorder keyboard control\n"
        "  r: start recording\n"
        "  c: stop current episode\n"
        "  s: save stopped episode\n"
        "  q: discard stopped episode\n"
        "  x: exit keyboard control\n",
        flush=True,
    )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()

    try:
        node.wait_for_services()
        print_help()

        with raw_terminal():
            while rclpy.ok():
                key = read_key()
                if key is None:
                    rclpy.spin_once(node, timeout_sec=0)
                    continue

                if key == "x":
                    break

                if key in KEY_TO_SERVICE:
                    node.call_service(key)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
