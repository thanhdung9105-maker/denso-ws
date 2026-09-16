#!/usr/bin/env python3
import sys
import os
import time
import argparse

if "RMW_IMPLEMENTATION" not in os.environ:
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

def main():
    parser = argparse.ArgumentParser(description="Denso VS-6556 Link 1-5 Joint Demo Controller")
    parser.add_argument("--stop", action="store_true", help="Stop the automated joint demo")
    parser.add_argument("--home", action="store_true", help="Send the arm to home 0°")
    parser.add_argument("--monitor", action="store_true", default=True, help="Monitor live joint movements in terminal")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = Node("denso_demo_cli")
    pub = node.create_publisher(String, "/denso/cmd", 10)

    # Allow publisher connection
    time.sleep(0.3)

    cmd = "demo"
    if args.stop:
        cmd = "stop"
    elif args.home:
        cmd = "home"

    msg = String()
    msg.data = cmd
    pub.publish(msg)

    if cmd == "demo":
        print("[INFO] Da gui lenh: Kich hoat chu trinh Link 1 -> Link 5 (2.0s / link, 25Hz smooth) tren RViz!")
    elif cmd == "stop":
        print("[INFO] Da gui lenh: Tam dung tu dong. Giu nguyen vi tri robot.")
    elif cmd == "home":
        print("[INFO] Da gui lenh: Thu toan bo canh tay ve vi tri Home 0°.")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
