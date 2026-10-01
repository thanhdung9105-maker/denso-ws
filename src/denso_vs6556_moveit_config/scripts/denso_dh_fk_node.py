#!/usr/bin/env python3
"""
DENSO VS-6556 EXTENDED D-H FORWARD KINEMATICS NODE (ROS 2)
Node tính toán và kiểm chứng Động học D-H Mở Rộng thời gian thực:
- Nhận dữ liệu góc khớp thực tế từ /joint_states
- Tính toán ma trận biến đổi thuần nhất từng khâu (base_link -> link_1..5 -> flange)
- Xuất PoseArray (/denso/dh_link_poses) cho các node điều khiển
- Xuất MarkerArray 3D (/denso/dh_markers) hiển thị hệ trục tọa độ & nhãn tọa độ trên RViz
- Xuất thông tin chuỗi JSON (/denso/dh_kinematics_info)
"""

import sys
import os
import math
import json
import numpy as np

# ROS 2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

# Import dynamics engine & Extended DH definitions
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    from denso_dynamics_engine import DensoDynamicsEngine, EXTENDED_DH_PARAMS, extended_dh_matrix
except ImportError:
    # Fallback definition if run standalone
    EXTENDED_DH_PARAMS = [
        ("link_1", 0.2050,  0.0000,  0.0000, "Z", 0, "Base -> Link 1 (Flange quay J1 h=205mm)"),
        ("link_2", 0.1300, -0.0750, -0.0745, "Y", 1, "Link 1 -> Link 2 (Vai J2, lệch motor Y=-74.5mm)"),
        ("link_3", 0.2700,  0.0000,  0.0065, "Y", 2, "Link 2 -> Link 3 (Khuỷu J3, lệch khớp Y=+6.5mm)"),
        ("link_4", 0.1090,  0.0900,  0.0680, "Z", 3, "Link 3 -> Link 4 (Cẳng tay J4, bù triệt tiêu Y)"),
        ("link_5", 0.1860,  0.0000,  0.0000, "Y", 4, "Link 4 -> Link 5 (Tâm cổ tay Point P)"),
        ("flange", 0.0800,  0.0000,  0.0000, "Fixed", -1, "Link 5 -> Flange TCP (Mặt bích +80mm)"),
    ]
    def extended_dh_matrix(d, a, b, q_angle=0.0, axis="Z"):
        T_trans = np.eye(4)
        T_trans[:3, 3] = [a, b, d]
        T_rot = np.eye(4)
        if axis == "Z":
            c, s = np.cos(q_angle), np.sin(q_angle)
            T_rot[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        elif axis == "Y":
            c, s = np.cos(q_angle), np.sin(q_angle)
            T_rot[:3, :3] = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        return T_trans @ T_rot


def rot_matrix_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class DensoExtendedDHFKNode(Node):
    def __init__(self):
        super().__init__('denso_dh_fk_node')
        
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('verbose', False)
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.verbose = self.get_parameter('verbose').value
        
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        
        # Publishers
        self.pose_array_pub = self.create_publisher(PoseArray, '/denso/dh_link_poses', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/denso/dh_markers', 10)
        self.info_pub = self.create_publisher(String, '/denso/dh_kinematics_info', 10)
        
        # Subscriber to /joint_states
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        # Timer loop for kinematics calculation & publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.compute_and_publish_fk)
        
        self.get_logger().info("===================================================================")
        self.get_logger().info("  DENSO VS-6556 EXTENDED D-H FORWARD KINEMATICS NODE INITIALIZED   ")
        self.get_logger().info("  Publishing: /denso/dh_link_poses, /denso/dh_markers, /denso/dh_kinematics_info")
        self.get_logger().info("===================================================================")

    def joint_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        updated = False
        for i, jname in enumerate(self.joint_names):
            if jname in name_to_pos:
                self.current_joints[i] = name_to_pos[jname]
                updated = True

    def compute_and_publish_fk(self):
        q = self.current_joints
        
        # Step-by-step forward kinematics using Extended D-H convention
        T = np.eye(4)
        transforms = [T.copy()]  # T0 = Base
        matrices_H = []
        link_names = ["base_link"]
        
        for name, d, a, b, axis, j_idx, desc in EXTENDED_DH_PARAMS:
            q_val = float(q[j_idx]) if j_idx >= 0 else 0.0
            H_i = extended_dh_matrix(d, a, b, q_val, axis)
            T = T @ H_i
            matrices_H.append(H_i)
            transforms.append(T.copy())
            link_names.append(name)
        
        stamp = self.get_clock().now().to_msg()
        
        # 1. Publish PoseArray
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = stamp
        pose_array_msg.header.frame_id = "base_link"
        
        for T_i in transforms:
            p = Pose()
            p.position.x = float(T_i[0, 3])
            p.position.y = float(T_i[1, 3])
            p.position.z = float(T_i[2, 3])
            qx, qy, qz, qw = rot_matrix_to_quaternion(T_i[:3, :3])
            p.orientation.x = float(qx)
            p.orientation.y = float(qy)
            p.orientation.z = float(qz)
            p.orientation.w = float(qw)
            pose_array_msg.poses.append(p)
            
        self.pose_array_pub.publish(pose_array_msg)
        
        # 2. Publish RViz 3D Markers (Coordinate Frame Arrows + Text Labels)
        marker_array = MarkerArray()
        marker_id = 0
        
        for i, (lname, T_i) in enumerate(zip(link_names, transforms)):
            pos = T_i[:3, 3]
            R_i = T_i[:3, :3]
            
            # Text Marker
            text_marker = Marker()
            text_marker.header.stamp = stamp
            text_marker.header.frame_id = "base_link"
            text_marker.ns = "dh_link_labels"
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(pos[0])
            text_marker.pose.position.y = float(pos[1])
            text_marker.pose.position.z = float(pos[2] + 0.035)  # slightly above link center
            text_marker.scale.z = 0.025  # text size (25mm)
            text_marker.color.r = 1.0
            text_marker.color.g = 0.9
            text_marker.color.b = 0.2
            text_marker.color.a = 0.95
            text_marker.text = f"[{lname}]\nX:{pos[0]*1000:+.0f} Y:{pos[1]*1000:+.0f} Z:{pos[2]*1000:+.0f} mm"
            marker_array.markers.append(text_marker)
            
            # Sphere Marker for Link Origin Center
            sphere_marker = Marker()
            sphere_marker.header.stamp = stamp
            sphere_marker.header.frame_id = "base_link"
            sphere_marker.ns = "dh_joint_spheres"
            sphere_marker.id = marker_id
            marker_id += 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = float(pos[0])
            sphere_marker.pose.position.y = float(pos[1])
            sphere_marker.pose.position.z = float(pos[2])
            sphere_marker.scale.x = 0.020
            sphere_marker.scale.y = 0.020
            sphere_marker.scale.z = 0.020
            sphere_marker.color.r = 0.1
            sphere_marker.color.g = 0.8
            sphere_marker.color.b = 1.0
            sphere_marker.color.a = 0.8
            marker_array.markers.append(sphere_marker)
            
        self.marker_pub.publish(marker_array)
        
        # 3. Publish JSON Kinematics Info
        info_dict = {
            "timestamp": stamp.sec + stamp.nanosec * 1e-9,
            "joints_rad": [float(val) for val in q],
            "joints_deg": [float(math.degrees(val)) for val in q],
            "links": {}
        }
        for lname, T_i in zip(link_names, transforms):
            info_dict["links"][lname] = {
                "x_m": round(float(T_i[0, 3]), 4),
                "y_m": round(float(T_i[1, 3]), 4),
                "z_m": round(float(T_i[2, 3]), 4),
                "matrix_4x4": [[round(float(v), 5) for v in row] for row in T_i.tolist()]
            }
        
        json_msg = String()
        json_msg.data = json.dumps(info_dict)
        self.info_pub.publish(json_msg)
        
        if self.verbose:
            self.get_logger().info(f"DH FK [Flange]: X={transforms[-1][0,3]*1000:+.1f}, Y={transforms[-1][1,3]*1000:+.1f}, Z={transforms[-1][2,3]*1000:+.1f} mm")


def main(args=None):
    rclpy.init(args=args)
    node = DensoExtendedDHFKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
