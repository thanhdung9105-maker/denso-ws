#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

POSES = {
    'Home (Zero Position)': [0.0, 0.0, 0.0, 0.0, 0.0],
    'Ready (Working Stance)': [0.0, 0.5, 0.8, 0.0, -0.5],
    'Reach Forward': [0.0, 0.75, 0.5, 0.0, 0.2],
    'Pick Low (Table Ground)': [0.0, 1.05, 0.25, 0.0, -0.7],
    'Reach Side Left (90 deg)': [1.2, 0.4, 0.7, 0.0, -0.3],
    'Reach Side Right (-90 deg)': [-1.2, 0.4, 0.7, 0.0, -0.3],
}

SEQUENCE = [
    'Ready (Working Stance)',
    'Reach Forward',
    'Pick Low (Table Ground)',
    'Reach Side Left (90 deg)',
    'Reach Side Right (-90 deg)',
    'Home (Zero Position)',
]

def plan_and_execute_pose(node, client, pose_name, joint_positions, pipeline='ompl', planner='RRTConnectkConfigDefault'):
    node.get_logger().info(f"===> Moving to: {pose_name} using [{pipeline} / {planner}]")
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = 'arm'
    req.pipeline_id = pipeline
    req.planner_id = planner
    req.num_planning_attempts = 5
    req.allowed_planning_time = 3.0
    req.max_velocity_scaling_factor = 0.1
    req.max_acceleration_scaling_factor = 0.1

    c = Constraints()
    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
    for name, val in zip(joint_names, joint_positions):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(val)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)

    goal.request = req
    goal.planning_options.plan_only = False

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal was rejected by MoveGroup.")
        return False

    res_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    result = res_future.result().result
    if result.error_code.val == 1:
        node.get_logger().info(f"Pose reached successfully! Holding for 2 seconds...")
        time.sleep(3.0)
        return True
    else:
        node.get_logger().warn(f"Plan/Execution returned error code: {result.error_code.val}")
        return False

def main():
    rclpy.init()
    node = Node('denso_animation_demo')
    client = ActionClient(node, MoveGroup, '/move_action')
    node.get_logger().info("Connecting to MoveGroup action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("MoveGroup server not found. Please ensure MoveIt is running (denso_moveit).")
        return

    node.get_logger().info("Connected! Starting automatic pose animation sequence...")
    try:
        # Alternating between OMPL and Pilz PTP to demonstrate both pipelines
        pipelines = [
            ('ompl', 'RRTConnectkConfigDefault'),
            ('pilz_industrial_motion_planner', 'PTP'),
            ('ompl', 'RRTConnectkConfigDefault'),
            ('pilz_industrial_motion_planner', 'PTP'),
            ('ompl', 'RRTConnectkConfigDefault'),
            ('pilz_industrial_motion_planner', 'PTP'),
        ]

        for i, pose_name in enumerate(SEQUENCE):
            joint_positions = POSES[pose_name]
            pipeline, planner = pipelines[i % len(pipelines)]
            plan_and_execute_pose(node, client, pose_name, joint_positions, pipeline, planner)

        node.get_logger().info("Full animation sequence completed successfully!")
    except KeyboardInterrupt:
        node.get_logger().info("Demo interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
