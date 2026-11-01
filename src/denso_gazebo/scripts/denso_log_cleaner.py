#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
denso_log_cleaner.py
Bo loc lam sach log khoi dong mo phong Denso VS-6556 tren ROS 2 Jazzy & Gazebo Harmonic.
Tu dong loai bo cac dong WARN trung lap hoac tam thoi khong anh huong den hoat dong:
1. controller_manager: The resource manager is not yet initialized...
2. gz_ros_control: Waiting RM to load and initialize hardware...
3. GazeboSimSystem: Executor is not available during hardware component initialization...
4. controller_manager: Component GazeboSimSystem does not have read or write statistics...
5. rviz2 class_loader.impl: SEVERE WARNING namespace collision InteractiveMarkerDisplay...
"""
import sys
import re

PATTERNS = [
    re.compile(r"The resource manager is not yet initialized"),
    re.compile(r"Waiting RM to load and initialize hardware"),
    re.compile(r"Executor is not available during hardware component initialization"),
    re.compile(r"does not have read or write statistics initialized"),
    re.compile(r"No clock received, using time argument instead"),
    re.compile(r"class_loader\.impl:\s*SEVERE WARNING"),
    re.compile(r"at line \d+ in /opt/ros/jazzy/include/class_loader"),
]

def main():
    try:
        for line in sys.stdin:
            if any(p.search(line) for p in PATTERNS):
                continue
            sys.stdout.write(line)
            sys.stdout.flush()
    except (KeyboardInterrupt, BrokenPipeError):
        pass

if __name__ == "__main__":
    main()
