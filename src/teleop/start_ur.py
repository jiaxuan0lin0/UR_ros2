#!/usr/bin/env python3
"""Bring a UR robot to a state where direct RTDE control can connect."""

import argparse
import socket
import sys
import time


DEFAULT_ROBOT_IP = "192.168.1.102"
DEFAULT_DASHBOARD_PORT = 29999


class DashboardClient:
    def __init__(self, robot_ip, port, timeout_sec):
        self.robot_ip = robot_ip
        self.port = port
        self.timeout_sec = timeout_sec
        self.sock = None
        self.reader = None

    def __enter__(self):
        self.sock = socket.create_connection(
            (self.robot_ip, self.port),
            timeout=self.timeout_sec,
        )
        self.sock.settimeout(self.timeout_sec)
        self.reader = self.sock.makefile(
            "r",
            encoding="utf-8",
            errors="replace",
            newline="\n",
        )
        hello = self._read_reply()
        if hello:
            print(hello)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.reader is not None:
            self.reader.close()
        if self.sock is not None:
            self.sock.close()

    def _read_reply(self):
        line = self.reader.readline()
        return line.strip()

    def send(self, command, wait_sec=0.0):
        print(f"> {command}")
        self.sock.sendall((command + "\n").encode("utf-8"))
        if wait_sec > 0:
            time.sleep(wait_sec)
        reply = self._read_reply()
        print(reply)
        print()
        return reply


def contains(reply, text):
    return text.upper() in reply.upper()


def wait_for_reply(client, command, expected_text, timeout_sec, poll_sec):
    deadline = time.monotonic() + timeout_sec
    last_reply = ""
    while time.monotonic() <= deadline:
        last_reply = client.send(command)
        if contains(last_reply, expected_text):
            return True, last_reply
        time.sleep(poll_sec)
    return False, last_reply


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Power on a UR robot, release brakes, and verify it is ready "
            "for direct RTDE control."
        )
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_ROBOT_IP,
        help=f"UR controller IP address. Default: {DEFAULT_ROBOT_IP}",
    )
    parser.add_argument(
        "--dashboard-port",
        type=int,
        default=DEFAULT_DASHBOARD_PORT,
        help=f"UR Dashboard Server port. Default: {DEFAULT_DASHBOARD_PORT}",
    )
    parser.add_argument(
        "--connect-timeout-sec",
        type=float,
        default=5.0,
        help="Socket connect/read timeout in seconds. Default: 5.0",
    )
    parser.add_argument(
        "--state-timeout-sec",
        type=float,
        default=15.0,
        help="Timeout while waiting for RUNNING/NORMAL state. Default: 15.0",
    )
    parser.add_argument(
        "--poll-sec",
        type=float,
        default=0.5,
        help="Polling interval while waiting for robot state. Default: 0.5",
    )
    parser.add_argument(
        "--unlock-protective-stop",
        action="store_true",
        help=(
            "Send 'unlock protective stop' when safetymode is "
            "PROTECTIVE_STOP. Use only after checking the real robot area."
        ),
    )
    parser.add_argument(
        "--skip-power-on",
        action="store_true",
        help="Do not send 'power on'.",
    )
    parser.add_argument(
        "--skip-brake-release",
        action="store_true",
        help="Do not send 'brake release'.",
    )
    parser.add_argument(
        "--no-wait-running",
        action="store_true",
        help="Do not wait for final robotmode RUNNING and safetymode NORMAL.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        with DashboardClient(
            args.robot_ip,
            args.dashboard_port,
            args.connect_timeout_sec,
        ) as dashboard:
            remote_reply = dashboard.send("is in remote control")
            robotmode_reply = dashboard.send("robotmode")
            safetymode_reply = dashboard.send("safetymode")

            if not contains(remote_reply, "true"):
                print(
                    "Robot is not in Remote Control. Switch the teach "
                    "pendant to Remote Control first.",
                    file=sys.stderr,
                )
                return 2

            if contains(safetymode_reply, "PROTECTIVE_STOP"):
                if not args.unlock_protective_stop:
                    print(
                        "Robot is in PROTECTIVE_STOP. Check the real robot "
                        "area, then rerun with --unlock-protective-stop if "
                        "it is safe.",
                        file=sys.stderr,
                    )
                    return 3
                dashboard.send("unlock protective stop", wait_sec=1.0)
                ok, safetymode_reply = wait_for_reply(
                    dashboard,
                    "safetymode",
                    "NORMAL",
                    args.state_timeout_sec,
                    args.poll_sec,
                )
                if not ok:
                    print(
                        "Protective stop was requested to unlock, but "
                        f"safetymode did not become NORMAL: {safetymode_reply}",
                        file=sys.stderr,
                    )
                    return 4

            if not args.skip_power_on:
                dashboard.send("power on", wait_sec=3.0)

            if not args.skip_brake_release:
                dashboard.send("brake release", wait_sec=2.0)

            if not args.no_wait_running:
                ok, robotmode_reply = wait_for_reply(
                    dashboard,
                    "robotmode",
                    "RUNNING",
                    args.state_timeout_sec,
                    args.poll_sec,
                )
                if not ok:
                    print(
                        "Robot did not reach RUNNING mode: "
                        f"{robotmode_reply}",
                        file=sys.stderr,
                    )
                    return 5

                ok, safetymode_reply = wait_for_reply(
                    dashboard,
                    "safetymode",
                    "NORMAL",
                    args.state_timeout_sec,
                    args.poll_sec,
                )
                if not ok:
                    print(
                        "Robot did not reach NORMAL safety mode: "
                        f"{safetymode_reply}",
                        file=sys.stderr,
                    )
                    return 6
            else:
                dashboard.send("robotmode")
                dashboard.send("safetymode")

    except OSError as exc:
        print(
            f"Failed to connect to UR Dashboard Server at "
            f"{args.robot_ip}:{args.dashboard_port}: {exc}",
            file=sys.stderr,
        )
        return 1

    print("UR is ready for direct RTDE control.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
