#!/usr/bin/env python3.12
"""
rtf_monitor.py — Real-Time Factor monitor for Gazebo sim.

Polls /stats topic and reports current, average, min, and max RTF.

Usage:
    python3.12 scripts/rtf_monitor.py               # run until Ctrl+C
    python3.12 scripts/rtf_monitor.py --samples 20  # stop after 20 samples
    python3.12 scripts/rtf_monitor.py --interval 2  # poll every 2s (default: 1s)
    python3.12 scripts/rtf_monitor.py --once         # single snapshot, machine-readable
"""

import argparse
import os
import signal
import subprocess
import sys
import time


def read_rtf_once(timeout: float = 4.0) -> float | None:
    """Read one RTF value by comparing two /clock readings over 0.5s wall time.

    Uses ros2 topic echo /clock (pure Python ROS2 subprocess, no orphan risk).
    RTF = (sim_time_delta) / (wall_time_delta).
    """
    def _read_clock_sec(t_deadline: float) -> float | None:
        proc = subprocess.Popen(
            ["ros2", "topic", "echo", "--once", "/clock"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
            start_new_session=True,
        )
        try:
            sec = None
            nanosec = None
            while time.monotonic() < t_deadline:
                line = proc.stdout.readline()
                if not line:
                    break
                line = line.strip()
                if line.startswith("sec:"):
                    try:
                        sec = int(line.split(":")[1].strip())
                    except ValueError:
                        pass
                elif line.startswith("nanosec:"):
                    try:
                        nanosec = int(line.split(":")[1].strip())
                    except ValueError:
                        pass
                if sec is not None and nanosec is not None:
                    return sec + nanosec / 1e9
            return None
        finally:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
            try:
                proc.wait(timeout=1.0)
            except Exception:
                pass

    t0_wall = time.monotonic()
    sim0 = _read_clock_sec(t0_wall + timeout / 2)
    if sim0 is None:
        return None

    wait = 0.5
    time.sleep(wait)

    t1_wall = time.monotonic()
    sim1 = _read_clock_sec(t1_wall + timeout / 2)
    if sim1 is None:
        return None

    wall_delta = t1_wall - t0_wall
    sim_delta = sim1 - sim0
    if wall_delta <= 0:
        return None
    return sim_delta / wall_delta


def main():
    parser = argparse.ArgumentParser(description="Monitor Gazebo RTF")
    parser.add_argument("--samples", type=int, default=0,
                        help="Stop after N samples (0 = run forever)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between samples (default: 1.0)")
    parser.add_argument("--once", action="store_true",
                        help="Print single line and exit (for scripting)")
    args = parser.parse_args()

    if args.once:
        rtf = read_rtf_once()
        if rtf is None:
            print("ERROR: could not read RTF", file=sys.stderr)
            sys.exit(1)
        print(f"{rtf:.3f}")
        return

    samples: list[float] = []
    print(f"{'Sample':>7}  {'Current':>8}  {'Average':>8}  {'Min':>8}  {'Max':>8}")
    print("-" * 48)

    n = 0
    try:
        while True:
            rtf = read_rtf_once()
            if rtf is not None:
                samples.append(rtf)
                n += 1
                avg = sum(samples) / len(samples)
                lo = min(samples)
                hi = max(samples)
                status = ""
                if rtf < 0.8:
                    status = "  ⚠ LOW"
                print(f"{n:>7}  {rtf:>8.3f}  {avg:>8.3f}  {lo:>8.3f}  {hi:>8.3f}{status}")
            else:
                print(f"{'?':>7}  {'--':>8}  (sim not reachable)")

            if args.samples and n >= args.samples:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass

    if samples:
        print("-" * 48)
        avg = sum(samples) / len(samples)
        lo = min(samples)
        hi = max(samples)
        print(f"{'FINAL':>7}  {'':>8}  {avg:>8.3f}  {lo:>8.3f}  {hi:>8.3f}  ({len(samples)} samples)")


if __name__ == "__main__":
    main()
