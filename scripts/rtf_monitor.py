#!/usr/bin/env python3.12
"""
rtf_monitor.py — Real-Time Factor monitor for Gazebo sim.

Reads real_time_factor directly from gz topic /stats.

Usage:
    python3.12 scripts/rtf_monitor.py               # run until Ctrl+C
    python3.12 scripts/rtf_monitor.py --samples 20  # stop after 20 samples
    python3.12 scripts/rtf_monitor.py --interval 2  # print every 2 msgs (default: 1)
    python3.12 scripts/rtf_monitor.py --once         # single snapshot, machine-readable
"""

import argparse
import os
import select
import subprocess
import sys
import time


def _open_stats() -> subprocess.Popen:
    return subprocess.Popen(
        ["gz", "topic", "-e", "-t", "/stats"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        start_new_session=True,
    )


def _kill(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), 9)
    except (ProcessLookupError, OSError):
        pass
    try:
        proc.wait(timeout=1.0)
    except Exception:
        pass


def read_rtf_once(timeout: float = 5.0) -> float | None:
    """Return the current real_time_factor from /stats, or None on timeout."""
    proc = _open_stats()
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            ready, _, _ = select.select([proc.stdout], [], [], remaining)
            if not ready:
                break
            line = proc.stdout.readline()
            if not line:
                break
            if line.strip().startswith("real_time_factor:"):
                try:
                    return float(line.split(":")[1].strip())
                except ValueError:
                    pass
        return None
    finally:
        _kill(proc)


def main():
    parser = argparse.ArgumentParser(description="Monitor Gazebo RTF via /stats")
    parser.add_argument("--samples", type=int, default=0,
                        help="Stop after N samples (0 = run forever)")
    parser.add_argument("--interval", type=int, default=1,
                        help="Print every N messages from /stats (default: 1)")
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

    proc = _open_stats()
    try:
        msg_count = 0
        n = 0
        while True:
            ready, _, _ = select.select([proc.stdout], [], [], 5.0)
            if not ready:
                print(f"{'?':>7}  {'--':>8}  (sim not reachable)")
                # Reopen — gz may have gone away and come back.
                _kill(proc)
                proc = _open_stats()
                continue

            line = proc.stdout.readline()
            if not line:
                break
            if not line.strip().startswith("real_time_factor:"):
                continue

            msg_count += 1
            if msg_count % args.interval != 0:
                continue

            try:
                rtf = float(line.split(":")[1].strip())
            except ValueError:
                continue

            samples.append(rtf)
            n += 1
            avg = sum(samples) / len(samples)
            status = "  ⚠ LOW" if rtf < 0.8 else ""
            print(f"{n:>7}  {rtf:>8.3f}  {avg:>8.3f}  {min(samples):>8.3f}  {max(samples):>8.3f}{status}")

            if args.samples and n >= args.samples:
                break
    except KeyboardInterrupt:
        pass
    finally:
        _kill(proc)

    if samples:
        print("-" * 48)
        avg = sum(samples) / len(samples)
        print(f"{'FINAL':>7}  {'':>8}  {avg:>8.3f}  {min(samples):>8.3f}  {max(samples):>8.3f}  ({len(samples)} samples)")


if __name__ == "__main__":
    main()
