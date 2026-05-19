#!/usr/bin/env python3
import argparse
import math
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parent
try:
    sys.path.remove(str(SCRIPT_DIR))
except ValueError:
    pass
sys.path.insert(0, str(SRC_DIR))

from teleop.xr_input import XrInput


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    index = (len(values) - 1) * pct / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return values[lower]
    weight = index - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure XRoboToolkit controller stream update rate and stalls."
    )
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--poll-hz", type=float, default=1000.0)
    parser.add_argument("--warn-gap-sec", type=float, default=0.05)
    parser.add_argument(
        "--pose-source",
        choices=("left_controller", "right_controller", "headset"),
        default="right_controller",
        help="Pose to read each poll, so the SDK path matches teleop more closely.",
    )
    args = parser.parse_args()

    duration_sec = max(0.1, args.duration_sec)
    poll_dt = 1.0 / max(1.0, args.poll_hz)
    warn_gap_sec = max(0.0, args.warn_gap_sec)

    xr = XrInput()
    try:
        print(
            f"Measuring XR stream for {duration_sec:.1f}s "
            f"at up to {1.0 / poll_dt:.0f}Hz polling..."
        )
        start_wall = time.monotonic()
        last_wall = start_wall
        last_ts = None
        update_wall_times: list[float] = []
        update_ts_values: list[float] = []
        stale_polls = 0
        invalid_pose_polls = 0

        while True:
            now = time.monotonic()
            if now - start_wall >= duration_sec:
                break

            try:
                pose = xr.get_pose(args.pose_source)
                if pose is None or not all(math.isfinite(float(v)) for v in pose):
                    invalid_pose_polls += 1
            except Exception:
                invalid_pose_polls += 1

            ts = xr.timestamp_sec()
            if ts <= 0.0:
                stale_polls += 1
            elif last_ts is None or ts != last_ts:
                update_wall_times.append(now)
                update_ts_values.append(ts)
                last_ts = ts
            else:
                stale_polls += 1

            elapsed = now - last_wall
            sleep_sec = poll_dt - elapsed
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
            last_wall = time.monotonic()

        total_wall = time.monotonic() - start_wall
    finally:
        xr.close()

    if len(update_wall_times) < 2:
        print("No valid XR timestamp updates observed.")
        print("Check XRoboToolkit PC Service and headset client connection first.")
        return 2

    wall_intervals = [
        b - a for a, b in zip(update_wall_times, update_wall_times[1:])
    ]
    sdk_intervals = [
        b - a for a, b in zip(update_ts_values, update_ts_values[1:])
        if b >= a
    ]
    intervals = sorted(wall_intervals)
    effective_hz = (len(update_wall_times) - 1) / max(
        1.0e-9, update_wall_times[-1] - update_wall_times[0]
    )
    gap_counts = Counter()
    for gap in wall_intervals:
        if gap >= 0.2:
            gap_counts[">=200ms"] += 1
        elif gap >= 0.1:
            gap_counts[">=100ms"] += 1
        elif gap >= 0.05:
            gap_counts[">=50ms"] += 1

    print(f"Observed updates: {len(update_wall_times)}")
    print(f"Measured duration: {total_wall:.3f}s")
    print(f"Effective update rate: {effective_hz:.1f} Hz")
    print(
        "Wall interval: "
        f"mean={statistics.fmean(wall_intervals) * 1000.0:.1f}ms, "
        f"p50={percentile(intervals, 50) * 1000.0:.1f}ms, "
        f"p95={percentile(intervals, 95) * 1000.0:.1f}ms, "
        f"p99={percentile(intervals, 99) * 1000.0:.1f}ms, "
        f"max={max(wall_intervals) * 1000.0:.1f}ms"
    )
    if sdk_intervals:
        sdk_sorted = sorted(sdk_intervals)
        print(
            "SDK timestamp step: "
            f"p50={percentile(sdk_sorted, 50) * 1000.0:.1f}ms, "
            f"p95={percentile(sdk_sorted, 95) * 1000.0:.1f}ms, "
            f"max={max(sdk_intervals) * 1000.0:.1f}ms"
        )
    print(
        "Large gaps: "
        f">=50ms={gap_counts['>=50ms']}, "
        f">=100ms={gap_counts['>=100ms']}, "
        f">=200ms={gap_counts['>=200ms']}"
    )
    print(f"Stale/zero timestamp polls: {stale_polls}")
    print(f"Invalid pose polls: {invalid_pose_polls}")

    large_gaps = [gap for gap in wall_intervals if gap >= warn_gap_sec]
    if large_gaps:
        print(
            f"Warning: {len(large_gaps)} gaps >= {warn_gap_sec * 1000.0:.0f}ms; "
            "this can feel like teleop stutter."
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
