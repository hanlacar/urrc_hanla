#!/usr/bin/env python3
"""Closed-loop check for the DR CSV follower and MCU steering conventions."""

import argparse
import csv
import math
from pathlib import Path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(path):
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return [{"x": float(r["dr_x_m"]), "y": float(r["dr_y_m"]),
             "direction": 1 if int(float(r.get("direction", 1))) >= 0 else -1}
            for r in rows]


def load_all_a_network(path):
    order = ("START_A", "COMMON_1", "T_foword", "T_A", "COMMON_2",
             "V_foword", "V_A", "END_common", "END_AA")
    with path.open(encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    selected = []
    for segment in order:
        part = [r for r in rows if r["segment_id"] == segment]
        if not part:
            raise RuntimeError(f"missing network segment: {segment}")
        selected.extend({
            "x": float(r["x_m"]), "y": float(r["y_m"]),
            "direction": 1 if int(float(r["direction"])) >= 0 else -1,
        } for r in part)
    return selected


def simulate(points, lateral=0.0, yaw_deg=0.0, dt=0.05):
    wheelbase = 0.73
    lookahead = 0.80
    max_steer = 22.0
    filter_alpha = 0.65
    max_rate = 45.0
    recovery_cte = 0.35
    goal_tol = 0.30
    x, y = points[0]["x"], points[0]["y"] + lateral
    route_yaw = math.atan2(points[4]["y"] - points[0]["y"],
                           points[4]["x"] - points[0]["x"])
    yaw = route_yaw + math.radians(yaw_deg)
    cursor = 0
    steer_ros = 0.0
    max_cte = 0.0
    recovery_steps = 0
    for step in range(200000):
        end = min(len(points), cursor + 80)
        nearest = min(range(cursor, end), key=lambda i: math.hypot(points[i]["x"]-x, points[i]["y"]-y))
        cursor = max(cursor, nearest)
        cte = math.hypot(points[cursor]["x"]-x, points[cursor]["y"]-y)
        max_cte = max(max_cte, cte)
        if cte > 2.0:
            return False, step*dt, max_cte, cte, cursor
        if cursor >= len(points)-2 and math.hypot(points[-1]["x"]-x, points[-1]["y"]-y) <= goal_tol:
            return True, step*dt, max_cte, cte, cursor
        direction = points[cursor]["direction"]
        target = cursor
        for i in range(cursor, len(points)):
            if points[i]["direction"] != direction:
                break
            target = i
            if math.hypot(points[i]["x"]-x, points[i]["y"]-y) >= lookahead:
                break
        dx, dy = points[target]["x"]-x, points[target]["y"]-y
        lx = math.cos(yaw)*dx + math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx + math.cos(yaw)*dy
        if direction < 0:
            lx, ly = -lx, -ly
        ld = max(0.05, math.hypot(lx, ly))
        curvature = 2.0*ly/(ld*ld)
        if direction < 0:
            curvature = -curvature
        target_steer = math.degrees(math.atan(wheelbase * curvature))
        target_steer = max(-max_steer, min(max_steer, target_steer))
        filtered = steer_ros + filter_alpha*(target_steer-steer_ros)
        delta = max_rate*dt
        steer_ros = max(steer_ros-delta, min(steer_ros+delta, filtered))

        # follower steering_sign=-1 -> MCU +right; firmware W time position
        # +right -> bridge ROS bicycle steering.  The two sign changes cancel.
        mcu_right_deg = -steer_ros
        physical_ros_deg = -mcu_right_deg
        speed = (0.229 if cte >= recovery_cte else 0.455) * direction
        x += speed*math.cos(yaw)*dt
        y += speed*math.sin(yaw)*dt
        yaw = wrap(yaw + speed*math.tan(math.radians(physical_ros_deg))/wheelbase*dt)
        recovery_steps += cte >= recovery_cte
    return False, 200000*dt, max_cte, cte, cursor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("routes", type=Path, nargs="?")
    ap.add_argument("--all-a-network", type=Path)
    args = ap.parse_args()
    scenarios = ((0.0, 0.0), (0.5, 0.0), (-0.5, 0.0), (0.8, 8.0), (-0.8, -8.0))
    failed = False
    cases = []
    if args.all_a_network:
        cases.append(("all_a", load_all_a_network(args.all_a_network)))
    elif args.routes:
        cases.extend((path.stem, load(path)) for path in sorted(args.routes.glob("*_dr.csv")))
    else:
        ap.error("routes or --all-a-network is required")
    for name, points in cases:
        for lateral, yaw in scenarios:
            ok, seconds, max_cte, final_cte, cursor = simulate(points, lateral, yaw)
            print(f"{name:8s} offset={lateral:+.1f}m yaw={yaw:+.0f}deg "
                  f"{'PASS' if ok else 'FAIL'} t={seconds:.1f}s "
                  f"max_cte={max_cte:.2f}m final={final_cte:.2f}m idx={cursor}")
            failed |= not ok
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
