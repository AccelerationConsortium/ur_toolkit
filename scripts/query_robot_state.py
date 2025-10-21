#!/usr/bin/env python3
from pathlib import Path
import sys

# add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from ur_toolkit.config_manager import config
import rtde_receive

robot_ip = config.get("robot.ip_address", "192.168.0.10")
print("Connecting to robot at", robot_ip)
rr = rtde_receive.RTDEReceiveInterface(robot_ip)
try:
    pose = rr.getActualTCPPose()
    joints = rr.getActualQ()
    print("Actual TCP pose:", pose)
    print("Actual joint positions:", joints)
finally:
    try:
        rr.disconnect()
    except Exception:
        pass
