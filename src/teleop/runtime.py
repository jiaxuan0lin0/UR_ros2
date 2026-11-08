import ctypes
import os


def configure_low_latency_runtime(priority: int, lock_memory: bool):
    if lock_memory:
        try:
            libc = ctypes.CDLL("libc.so.6", use_errno=True)
            mcl_current = 1
            mcl_future = 2
            if libc.mlockall(mcl_current | mcl_future) != 0:
                errno = ctypes.get_errno()
                raise OSError(errno, os.strerror(errno))
            print("Locked teleop process memory with mlockall.")
        except Exception as exc:
            print(f"Warning: failed to lock memory for low latency: {exc}")

    priority = int(priority)
    if priority <= 0:
        return
    try:
        os.sched_setscheduler(
            0,
            os.SCHED_FIFO,
            os.sched_param(priority),
        )
        print(f"Teleop process scheduler set to SCHED_FIFO priority {priority}.")
    except PermissionError as exc:
        print(
            "Warning: failed to set real-time scheduler. "
            "Configure realtime limits for this user first "
            f"(requested SCHED_FIFO priority {priority}): {exc}"
        )
    except Exception as exc:
        print(f"Warning: failed to set real-time scheduler: {exc}")


def configure_xr_service_runtime(priority: int):
    priority = int(priority)
    if priority <= 0:
        return

    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/comm", "r") as f:
                comm = f.read().strip()
        except OSError:
            continue
        if comm == "RoboticsService":
            pids.append(int(name))

    if not pids:
        print(
            "Warning: XR service process was not found; "
            "start XRoboToolkit PC Service before teleop."
        )
        return

    for pid in pids:
        try:
            os.sched_setscheduler(
                pid,
                os.SCHED_FIFO,
                os.sched_param(priority),
            )
            print(f"XR service PID {pid} set to SCHED_FIFO priority {priority}.")
        except PermissionError as exc:
            print(
                "Warning: failed to set XR service real-time scheduler "
                f"for PID {pid}: {exc}"
            )
        except Exception as exc:
            print(f"Warning: failed to boost XR service PID {pid}: {exc}")
