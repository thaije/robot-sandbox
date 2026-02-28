#!/usr/bin/env python3.12
"""test_sensors.py — Verify IMU + LiDAR sensors for DerpBot.

Two tests:
  1. IMU motion test: drive forward 0.5 s → backward 0.5 s → CW 0.5 s.
     Checks angular velocity sign, linear acceleration direction, gravity.
  2. Scan obstacle test: analyse 360° scan at spawn pose; expect walls/furniture
     to be visible at close range in several quadrants.

Run with scenario already running:
    ./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless &
    sleep 18
    python3.12 scripts/test_sensors.py
"""
from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan

ROBOT = "derpbot_0"
PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
WARN = "\033[33mWARN\033[0m"

# ── helpers ───────────────────────────────────────────────────────────────────

def _fmt(val: float, unit: str = "") -> str:
    return f"{val:+.4f}{unit}"


def _stats(samples: list[float]) -> dict:
    if not samples:
        return {"n": 0, "mean": 0.0, "min": 0.0, "max": 0.0, "std": 0.0}
    n = len(samples)
    mean = sum(samples) / n
    var = sum((x - mean) ** 2 for x in samples) / n
    return {
        "n": n,
        "mean": mean,
        "min": min(samples),
        "max": max(samples),
        "std": math.sqrt(var),
    }


# ── rclpy node ────────────────────────────────────────────────────────────────

class SensorTestNode(Node):
    def __init__(self):
        super().__init__("sensor_test")

        qos_be = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=20)

        # IMU samples: list of (phase_name, ax, ay, az, wx, wy, wz)
        self._imu_buf: list[tuple] = []
        self._imu_phase = "init"
        self._imu_lock = threading.Lock()

        # Latest scan
        self._scan: LaserScan | None = None
        self._scan_event = threading.Event()

        self.create_subscription(Imu, f"/{ROBOT}/imu", self._imu_cb, qos_be)
        self.create_subscription(LaserScan, f"/{ROBOT}/scan", self._scan_cb, qos_rel)

        self._pub = self.create_publisher(Twist, f"/{ROBOT}/cmd_vel", 10)

    def _imu_cb(self, msg: Imu):
        with self._imu_lock:
            self._imu_buf.append((
                self._imu_phase,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ))

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg
        self._scan_event.set()

    def set_phase(self, name: str):
        with self._imu_lock:
            self._imu_phase = name

    def imu_samples(self, phase: str) -> dict[str, list[float]]:
        with self._imu_lock:
            rows = [r for r in self._imu_buf if r[0] == phase]
        out: dict[str, list[float]] = defaultdict(list)
        for _, ax, ay, az, wx, wy, wz in rows:
            out["ax"].append(ax)
            out["ay"].append(ay)
            out["az"].append(az)
            out["wx"].append(wx)
            out["wy"].append(wy)
            out["wz"].append(wz)
        return dict(out)

    def send_cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self._pub.publish(t)

    def stop(self):
        for _ in range(8):
            self.send_cmd(0.0, 0.0)
            time.sleep(0.05)


# ── run executor in background thread ────────────────────────────────────────

def _spin(node: Node, stop_event: threading.Event):
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.01)


# ── test 1: IMU vs motion ─────────────────────────────────────────────────────

def test_imu_motion(node: SensorTestNode) -> bool:
    print("\n══════════════════════════════════════════")
    print("  TEST 1: IMU consistency with movement")
    print("══════════════════════════════════════════")

    DRIVE_DUR = 0.5   # seconds per phase
    FWD_VX    = 0.30  # m/s
    CW_WZ     = -0.8  # rad/s  (negative = clockwise = right-turn)

    # ── Phase 0: rest baseline ────────────────────────────────────────────────
    print("\n[0] Resting baseline (1 s) …")
    node.set_phase("rest")
    time.sleep(1.0)

    rest = node.imu_samples("rest")
    rs_ax = _stats(rest.get("ax", []))
    rs_az = _stats(rest.get("az", []))
    rs_wz = _stats(rest.get("wz", []))
    print(f"    ax: mean={_fmt(rs_ax['mean'],' m/s²')}  std={rs_ax['std']:.4f}  "
          f"n={rs_ax['n']}")
    print(f"    az: mean={_fmt(rs_az['mean'],' m/s²')}  (expect ≈ +9.8)")
    print(f"    wz: mean={_fmt(rs_wz['mean'],' rad/s')}  std={rs_wz['std']:.5f}")

    ok_gravity = 9.0 < rs_az["mean"] < 10.5
    ok_ax_rest = abs(rs_ax["mean"]) < 0.5
    ok_wz_rest = abs(rs_wz["mean"]) < 0.05
    print(f"    gravity check (az≈9.8):  {PASS if ok_gravity else FAIL}")
    print(f"    no lateral accel at rest: {PASS if ok_ax_rest else FAIL}")
    print(f"    no rotation at rest:      {PASS if ok_wz_rest else FAIL}")

    # ── Phase 1: forward ──────────────────────────────────────────────────────
    print(f"\n[1] Forward {FWD_VX} m/s for {DRIVE_DUR} s …")
    node.set_phase("fwd")
    t0 = time.monotonic()
    while time.monotonic() - t0 < DRIVE_DUR:
        node.send_cmd(FWD_VX, 0.0)
        time.sleep(0.05)
    node.stop()
    time.sleep(0.3)

    fwd = node.imu_samples("fwd")
    fwd_ax = _stats(fwd.get("ax", []))
    fwd_az = _stats(fwd.get("az", []))
    fwd_wz = _stats(fwd.get("wz", []))
    print(f"    ax: mean={_fmt(fwd_ax['mean'],' m/s²')}  min={fwd_ax['min']:+.3f}"
          f"  max={fwd_ax['max']:+.3f}  n={fwd_ax['n']}")
    print(f"    az: mean={_fmt(fwd_az['mean'],' m/s²')}")
    print(f"    wz: mean={_fmt(fwd_wz['mean'],' rad/s')}  (expect ≈ 0)")

    # During 0.5s drive: should see an acceleration spike at start (ax>0)
    # and possibly a deceleration spike at the end (ax<0 after stop).
    # Gravity should still be present.
    ok_fwd_gravity = 9.0 < fwd_az["mean"] < 10.5
    ok_fwd_ax_nonzero = fwd_ax["max"] > 0.05   # some forward accel transient
    ok_fwd_wz_small = abs(fwd_wz["mean"]) < 0.3
    print(f"    gravity maintained during fwd: {PASS if ok_fwd_gravity else FAIL}")
    print(f"    forward accel transient (ax>0 at some point): "
          f"{PASS if ok_fwd_ax_nonzero else WARN}")
    print(f"    no significant yaw during fwd: {PASS if ok_fwd_wz_small else FAIL}")

    # ── Phase 2: backward ─────────────────────────────────────────────────────
    print(f"\n[2] Backward {-FWD_VX} m/s for {DRIVE_DUR} s …")
    node.set_phase("bwd")
    t0 = time.monotonic()
    while time.monotonic() - t0 < DRIVE_DUR:
        node.send_cmd(-FWD_VX, 0.0)
        time.sleep(0.05)
    node.stop()
    time.sleep(0.3)

    bwd = node.imu_samples("bwd")
    bwd_ax = _stats(bwd.get("ax", []))
    bwd_az = _stats(bwd.get("az", []))
    bwd_wz = _stats(bwd.get("wz", []))
    print(f"    ax: mean={_fmt(bwd_ax['mean'],' m/s²')}  min={bwd_ax['min']:+.3f}"
          f"  max={bwd_ax['max']:+.3f}  n={bwd_ax['n']}")
    print(f"    az: mean={_fmt(bwd_az['mean'],' m/s²')}")

    ok_bwd_gravity = 9.0 < bwd_az["mean"] < 10.5
    ok_bwd_ax_nonzero = bwd_ax["min"] < -0.05   # backward accel transient
    ok_bwd_wz_small = abs(bwd_wz["mean"]) < 0.3
    print(f"    gravity maintained during bwd: {PASS if ok_bwd_gravity else FAIL}")
    print(f"    backward decel transient (ax<0 at some point): "
          f"{PASS if ok_bwd_ax_nonzero else WARN}")
    print(f"    no significant yaw during bwd: {PASS if ok_bwd_wz_small else FAIL}")

    # ── Phase 3: clockwise rotation ───────────────────────────────────────────
    print(f"\n[3] Clockwise rotation wz={CW_WZ} rad/s for {DRIVE_DUR} s …")
    node.set_phase("cw")
    t0 = time.monotonic()
    while time.monotonic() - t0 < DRIVE_DUR:
        node.send_cmd(0.0, CW_WZ)
        time.sleep(0.05)
    node.stop()
    time.sleep(0.3)

    cw = node.imu_samples("cw")
    cw_ax = _stats(cw.get("ax", []))
    cw_az = _stats(cw.get("az", []))
    cw_wz = _stats(cw.get("wz", []))
    print(f"    wz: mean={_fmt(cw_wz['mean'],' rad/s')}  min={cw_wz['min']:+.3f}"
          f"  max={cw_wz['max']:+.3f}  n={cw_wz['n']}")
    print(f"    ax: mean={_fmt(cw_ax['mean'],' m/s²')}  (expect ≈ 0)")
    print(f"    az: mean={_fmt(cw_az['mean'],' m/s²')}")

    ok_cw_sign = cw_wz["mean"] < -0.1      # negative = clockwise
    ok_cw_magnitude = abs(cw_wz["mean"]) > 0.3  # significant rotation
    ok_cw_gravity = 9.0 < cw_az["mean"] < 10.5
    ok_cw_ax_small = abs(cw_ax["mean"]) < 0.5
    print(f"    CW sign correct (wz<0):    {PASS if ok_cw_sign else FAIL}")
    print(f"    CW magnitude sensible:     {PASS if ok_cw_magnitude else FAIL}")
    print(f"    gravity during rotation:   {PASS if ok_cw_gravity else FAIL}")
    print(f"    no linear accel during CW: {PASS if ok_cw_ax_small else PASS}")

    all_ok = (ok_gravity and ok_ax_rest and ok_wz_rest
              and ok_fwd_gravity and ok_fwd_wz_small
              and ok_bwd_gravity and ok_bwd_wz_small
              and ok_cw_sign and ok_cw_magnitude and ok_cw_gravity)

    print(f"\n  IMU test overall: {PASS if all_ok else FAIL}")
    return all_ok


# ── test 2: scan obstacle detection ──────────────────────────────────────────

def test_scan_obstacles(node: SensorTestNode) -> bool:
    print("\n══════════════════════════════════════════")
    print("  TEST 2: LiDAR obstacle detection")
    print("══════════════════════════════════════════")

    print("\nWaiting for scan …")
    if not node._scan_event.wait(timeout=5.0):
        print(f"  {FAIL}: no scan received within 5 s")
        return False

    scan = node._scan
    ranges = list(scan.ranges)
    n = len(ranges)
    inc = scan.angle_increment
    min_a = scan.angle_min

    print(f"\n  Frame  : {scan.header.frame_id}")
    print(f"  Samples: {n}  angle [{math.degrees(scan.angle_min):.1f}°,"
          f" {math.degrees(scan.angle_max):.1f}°]  "
          f"range [{scan.range_min:.2f}, {scan.range_max:.2f}] m")

    # Classify each ray into one of 8 sectors (N/NE/E/SE/S/SW/W/NW)
    # Robot faces +x (forward) → 0 rad
    # Sectors centred at 0, π/4, π/2, 3π/4, ±π, -3π/4, -π/2, -π/4
    sector_labels = ["F(+x)", "FL", "L(+y)", "BL", "B(-x)", "BR", "R(-y)", "FR"]
    sector_counts  = [0] * 8
    sector_min_r   = [float("inf")] * 8
    sector_sum_r   = [0.0] * 8

    finite_count = 0
    for i, r in enumerate(ranges):
        if math.isinf(r) or math.isnan(r):
            continue
        if r < scan.range_min or r > scan.range_max:
            continue
        finite_count += 1
        angle = min_a + i * inc
        # normalise to [-π, π]
        angle = math.atan2(math.sin(angle), math.cos(angle))
        sec_idx = int((angle + math.pi + math.pi / 8) / (math.pi / 4)) % 8
        sector_counts[sec_idx] += 1
        sector_min_r[sec_idx] = min(sector_min_r[sec_idx], r)
        sector_sum_r[sec_idx] += r

    print(f"\n  Finite returns: {finite_count}/{n} ({100*finite_count/n:.1f}%)")
    print(f"\n  {'Sector':<8}  {'Rays':>5}  {'min_r':>7}  {'mean_r':>7}")
    print(f"  {'-'*8}  {'-'*5}  {'-'*7}  {'-'*7}")
    for i, lbl in enumerate(sector_labels):
        cnt = sector_counts[i]
        mn_r = sector_min_r[i] if cnt > 0 else float("nan")
        avg_r = sector_sum_r[i] / cnt if cnt > 0 else float("nan")
        bar = "█" * min(cnt // 10, 20)
        near_flag = " ◄ close!" if mn_r < 1.5 else ""
        print(f"  {lbl:<8}  {cnt:>5}  {mn_r:>7.2f}  {avg_r:>7.2f}  {bar}{near_flag}")

    # Assertions:
    # 1. Majority of rays return a finite hit (not open sky)
    # 2. At least 2 sectors have obstacles within 2 m (walls/furniture visible)
    # 3. Spawn (1,1): west wall (B=-x) ≈ 1m, south wall (R=-y) ≈ 1m
    ok_coverage = finite_count > n * 0.7
    near_sectors = sum(1 for r in sector_min_r if r < 2.0)
    ok_near = near_sectors >= 2

    # West wall: in "B(-x)" sector (idx 4), distance ≈ 1m
    bx_min = sector_min_r[4]  # B(-x)
    # South wall: in "R(-y)" sector (idx 6), distance ≈ 1m
    ry_min = sector_min_r[6]  # R(-y)
    ok_west_wall = bx_min < 2.0
    ok_south_wall = ry_min < 2.0

    print(f"\n  scan coverage >70%:         {PASS if ok_coverage else FAIL} "
          f"({finite_count}/{n})")
    print(f"  ≥2 sectors with obs <2m:    {PASS if ok_near else FAIL} "
          f"(got {near_sectors})")
    print(f"  West wall visible (<2m):    {PASS if ok_west_wall else WARN} "
          f"(min_r={bx_min:.2f}m)")
    print(f"  South wall visible (<2m):   {PASS if ok_south_wall else WARN} "
          f"(min_r={ry_min:.2f}m)")

    all_ok = ok_coverage and ok_near
    print(f"\n  Scan test overall: {PASS if all_ok else FAIL}")
    return all_ok


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print("╔══════════════════════════════════════════╗")
    print("║  DerpBot Sensor Test — IMU + LiDAR       ║")
    print("╚══════════════════════════════════════════╝")
    print(f"  Robot: {ROBOT}")

    rclpy.init()
    node = SensorTestNode()

    stop_event = threading.Event()
    spin_thread = threading.Thread(target=_spin, args=(node, stop_event), daemon=True)
    spin_thread.start()

    # Wait for IMU and scan to appear
    print("\nWaiting for IMU and scan topics (up to 8 s) …")
    t0 = time.monotonic()
    got_imu = False
    while time.monotonic() - t0 < 8.0:
        with node._imu_lock:
            got_imu = len(node._imu_buf) > 0
        if got_imu and node._scan is not None:
            break
        time.sleep(0.1)

    if not got_imu:
        print(f"  {FAIL}: IMU not publishing — is the sim running?")
        stop_event.set(); spin_thread.join(timeout=2.0)
        node.destroy_node(); rclpy.shutdown()
        return 1

    print("  Both sensors live — starting tests.\n")

    results = []
    try:
        results.append(test_imu_motion(node))
        results.append(test_scan_obstacles(node))
    finally:
        node.stop()
        stop_event.set()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    print("\n══════════════════════════════════════════")
    passed = sum(results)
    total = len(results)
    status = PASS if passed == total else FAIL
    print(f"  Overall: {status}  ({passed}/{total} tests passed)")
    print("══════════════════════════════════════════\n")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
