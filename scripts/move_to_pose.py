#!/usr/bin/env python3
"""
Move robot TCP to a specified 6-DOF pose.

Usage:
  python scripts/move_to_pose.py x y z rx ry rz [--speed S] [--accel A]

Example:
  python scripts/move_to_pose.py 0.02387 -0.42358 0.34418 -0.52124 -2.26638 1.60193 --speed 0.03 --accel 0.05
"""

import sys
from pathlib import Path
import argparse
import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import rtde_control
from ur_toolkit.config_manager import config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "pose", nargs=6, type=float, help="Target TCP pose (x y z rx ry rz)"
    )
    parser.add_argument("--speed", type=float, default=0.03, help="Linear speed m/s")
    parser.add_argument("--accel", type=float, default=0.05, help="Linear accel m/s^2")
    args = parser.parse_args()

    robot_ip = config.get("robot.ip_address", "192.168.0.10")
    print("Connecting to robot at", robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)

    target = np.array(args.pose)
    print("Moving to:", target, "speed=", args.speed, "accel=", args.accel)
    ok = rtde_c.moveL(target.tolist(), args.speed, args.accel)
    if ok:
        print("Move completed successfully")
    else:
        print("Move returned False or failed")

    rtde_c.disconnect()


if __name__ == "__main__":
    main()
