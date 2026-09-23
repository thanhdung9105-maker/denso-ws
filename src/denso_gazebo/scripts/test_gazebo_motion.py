#!/usr/bin/env python3
"""
Test Gazebo Motion script for Denso VS-6556
Sends JointTrajectory goals to /arm_controller/follow_joint_trajectory action
and verifies joint state feedback from Gazebo Sim.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import math

class DensoGazeboMotionTester(Node):
    def __init__(self):
        super().__init__('denso_gazebo_motion_tester')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.current_joint_positions = {}
        
        # Subscribe to joint states from Gazebo
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # FollowJointTrajectory action client
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Dang ket noi toi /arm_controller/follow_joint_trajectory...')

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.current_joint_positions[name] = pos

    def wait_for_joint_states(self, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.current_joint_positions) >= 5:
                return True
        return False

    def send_trajectory_goal(self, waypoints):
        """
        waypoints: list of tuples (positions_list, time_from_start_sec)
        """
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server /arm_controller/follow_joint_trajectory khong phan hoi!')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        for positions, time_sec in waypoints:
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            sec = int(time_sec)
            nanosec = int((time_sec - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Dang gui quy dao gom {len(waypoints)} diem moc toi Gazebo...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Quy dao bi tu choi!')
            return False

        self.get_logger().info('Quy dao duoc chap nhan. Dang theo doi chuyen dong...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('[THANH CONG] Hoan thanh chuyen dong chinh xac va on dinh!')
            return True
        else:
            self.get_logger().warn(f'Chuyen dong ket thuc voi ma loi: {result.result.error_code}')
            return False

def main():
    rclpy.init()
    tester = DensoGazeboMotionTester()

    # Wait for joint states
    tester.get_logger().info('Dang doi nhan du lieu tu /joint_states...')
    if not tester.wait_for_joint_states(timeout=15.0):
        tester.get_logger().error('Khong nhan duoc /joint_states tu Gazebo Sim!')
        tester.destroy_node()
        rclpy.shutdown()
        return

    tester.get_logger().info(f'Vi tri hien tai cac khop: {tester.current_joint_positions}')

    # Define test motion routine
    waypoints = [
        # Point 1: Home 0 deg
        ([0.0, 0.0, 0.0, 0.0, 0.0], 2.5),
        # Point 2: Working pose (Reach forward & tilt)
        ([0.5, 0.4, 0.6, -0.3, 0.3], 5.5),
        # Point 3: Side pose
        ([-0.6, -0.2, 0.7, 0.4, -0.4], 9.0),
        # Point 4: Return Home
        ([0.0, 0.0, 0.0, 0.0, 0.0], 12.5),
    ]

    success = tester.send_trajectory_goal(waypoints)
    
    # Final check of joint positions
    tester.wait_for_joint_states(timeout=2.0)
    tester.get_logger().info(f'Vi tri cuoi cung cua robot: {tester.current_joint_positions}')

    if success:
        print("\n=======================================================")
        print("  MÔ PHỎNG DENSO VS-6556 TRÊN GAZEBO CHẠY RẤT ỔN ĐỊNH!  ")
        print("=======================================================\n")
    else:
        print("\n[CANH BAO] Co loi xay ra trong qua trinh dieu khien quy dao.")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
