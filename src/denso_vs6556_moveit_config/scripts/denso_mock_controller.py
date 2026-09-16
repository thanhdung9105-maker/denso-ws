#!/usr/bin/env python3
"""
Denso VS-6556 Mock Controller:
- 10Hz stable publishing, synchronized with ROS clock.
- FIXED AT 0 DEG ON STARTUP: The robot remains completely stationary at all joints = 0.0 deg
  when launched, until explicitly instructed to move.
- Pure 1-direction motion per link: Link 1 -> Link 2 -> Link 3 -> Link 4 -> Link 5.
- Exactly 2.0s duration per link movement, with 1.0s observation pause.
- Smooth S-curve (cosine) interpolation.
- Supports both continuous 1-direction loop and single 1-direction pass ('once').
- MoveIt 2 FollowJointTrajectory action server support with auto-pause on manual planning.
- Commands via '/denso/cmd': 'demo' (loop), 'once' (run 1 way and hold), 'stop', 'home'.
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String

# Pure 1-direction cumulative deployment:
# Each link moves in 1 direction for 2.0s without any link reversing back and forth
DEMO_STEPS = [
    # (joint_index, description, target_positions)
    (0, "Link 1 (Base / Truc Z): Xoay sang +45 deg [1 chieu]", [0.785, 0.0, 0.0, 0.0, 0.0]),
    (1, "Link 2 (Vai / Truc Y): Gap nghieng toi truoc +45 deg [1 chieu]", [0.785, 0.785, 0.0, 0.0, 0.0]),
    (2, "Link 3 (Khuu tay / Truc Y): Gap khuu tay xuong +45 deg [1 chieu]", [0.785, 0.785, 0.785, 0.0, 0.0]),
    (3, "Link 4 (Cang tay / Forearm Roll): Xoay truc doc +90 deg [1 chieu]", [0.785, 0.785, 0.785, 1.57, 0.0]),
    (4, "Link 5 (Co tay / Wrist Pitch): Gap ngua co tay len +50 deg [1 chieu]", [0.785, 0.785, 0.785, 1.57, 0.87]),
    (-1, "Toan bo canh tay: Dong bo thu ve Home (0 deg)", [0.0, 0.0, 0.0, 0.0, 0.0]),
]

class DensoController(Node):
    def __init__(self):
        super().__init__('denso_mock_controller')

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        # Fixed at 0.0 degrees on launch
        self.current_positions = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Timing parameters
        self.link_duration = 2.0   # Exactly 2.0s movement per link
        self.pause_duration = 1.0  # 1.0s pause for observation

        # Demo State: Default FALSE so robot remains stationary at 0 degrees on startup!
        self.demo_enabled = False
        self.demo_loop = True
        self.demo_step_idx = 0
        self.demo_state = "IDLE"
        self.demo_state_start_time = time.monotonic()
        self.demo_start_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.demo_target_pos = list(DEMO_STEPS[0][2])
        self.external_active = False

        self.lock = threading.Lock()
        # Depth 1 keeps TF latency zero
        self.js_pub = self.create_publisher(JointState, '/joint_states', 1)

        self.cmd_sub = self.create_subscription(
            String,
            '/denso/cmd',
            self.cmd_callback,
            10
        )

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.running = True
        self.loop_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.loop_thread.start()

        self.get_logger().info('Denso Controller ready: Tat ca cac link co dinh tai vi tri goc 0 deg.')
        self.get_logger().info('Go lenh "denso_joints" hoac "denso_once" de chay chuyen dong tu dong.')

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Nhan lenh: {cmd}')
        with self.lock:
            if cmd in ['demo', 'play', 'start', 'resume', 'loop']:
                self.demo_enabled = True
                self.demo_loop = True
                self.external_active = False
                self.demo_step_idx = 0
                self.start_next_demo_step()
            elif cmd in ['once', 'single']:
                self.demo_enabled = True
                self.demo_loop = False
                self.external_active = False
                self.demo_step_idx = 0
                self.start_next_demo_step()
            elif cmd in ['stop', 'pause']:
                self.demo_enabled = False
                self.demo_state = "PAUSE"
                self.get_logger().info('Da dung tu dong. Giu nguyen vi tri.')
            elif cmd == 'home':
                self.demo_enabled = False
                self.demo_state = "MOVE"
                self.demo_start_pos = list(self.current_positions)
                self.demo_target_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
                self.demo_state_start_time = time.monotonic()
                self.get_logger().info('Canh tay dang ve Home 0 deg...')
            elif cmd.startswith('goto'):
                try:
                    parts = [float(x) for x in cmd.split()[1:]]
                    if len(parts) == 5:
                        self.demo_enabled = False
                        self.demo_state = "MOVE"
                        self.demo_start_pos = list(self.current_positions)
                        self.demo_target_pos = list(parts)
                        self.demo_state_start_time = time.monotonic()
                        self.get_logger().info(f'Di chuyen toi vi tri khop: {parts}')
                except Exception as e:
                    self.get_logger().error(f'Loi parse goto: {e}')

    def start_next_demo_step(self):
        step = DEMO_STEPS[self.demo_step_idx]
        self.demo_start_pos = list(self.current_positions)
        self.demo_target_pos = list(step[2])
        self.demo_state = "MOVE"
        self.demo_state_start_time = time.monotonic()
        self.get_logger().info(f'[2s/LINK] {step[1]}')

    def goal_callback(self, goal_request):
        self.get_logger().info('Chap nhan yeu cau quy dao tu MoveGroup / RViz.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        points = trajectory.points
        if not points:
            if goal_handle.is_active:
                goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        name_map = {name: i for i, name in enumerate(self.joint_names)}
        traj_indices = [name_map[n] for n in trajectory.joint_names if n in name_map]

        traj_dur = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
        target_dur = max(1.5, min(4.0, traj_dur)) if traj_dur > 0.3 else 0.5

        self.get_logger().info(f'Thuc thi quy dao MoveGroup: {len(points)} waypoints trong {target_dur:.2f}s...')

        with self.lock:
            self.external_active = True
            # Automatically pause demo loop so manual MoveGroup execution is held
            self.demo_enabled = False

        start_time = time.monotonic()
        time_scale = traj_dur / target_dur if target_dur > 1e-6 else 1.0

        while rclpy.ok() and self.running:
            now = time.monotonic()
            elapsed_wall = now - start_time

            if goal_handle.is_cancel_requested:
                if goal_handle.is_active:
                    goal_handle.canceled()
                with self.lock:
                    self.external_active = False
                res = FollowJointTrajectory.Result()
                return res

            if elapsed_wall >= target_dur:
                final_pt = points[-1]
                with self.lock:
                    for i, idx in enumerate(traj_indices):
                        if i < len(final_pt.positions):
                            self.current_positions[idx] = float(final_pt.positions[i])
                break

            elapsed = elapsed_wall * time_scale
            for k in range(len(points) - 1):
                t0 = points[k].time_from_start.sec + points[k].time_from_start.nanosec * 1e-9
                t1 = points[k + 1].time_from_start.sec + points[k + 1].time_from_start.nanosec * 1e-9
                if t0 <= elapsed <= t1:
                    dt = t1 - t0
                    lin = (elapsed - t0) / dt if dt > 1e-6 else 0.0
                    lin = max(0.0, min(1.0, lin))
                    s_curve = 0.5 * (1.0 - math.cos(math.pi * lin))

                    with self.lock:
                        for i, idx in enumerate(traj_indices):
                            if i < len(points[k].positions) and i < len(points[k + 1].positions):
                                p0 = points[k].positions[i]
                                p1 = points[k + 1].positions[i]
                                self.current_positions[idx] = float(p0 + s_curve * (p1 - p0))
                    break

            time.sleep(0.05)

        if goal_handle.is_active:
            goal_handle.succeed()
            self.get_logger().info('Hoan thanh quy dao tu MoveGroup! Robot giu nguyen tai dich.')

        with self.lock:
            self.external_active = False

        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return res

    def control_loop(self):
        """Dedicated high-precision 10Hz loop: perfect match for RViz 10fps."""
        period = 0.10  # 10Hz (100ms)
        while self.running and rclpy.ok():
            t_start = time.monotonic()
            now = t_start

            with self.lock:
                if not self.external_active and self.demo_enabled:
                    if self.demo_state == "MOVE":
                        elapsed = now - self.demo_state_start_time
                        if elapsed >= self.link_duration:
                            self.current_positions = list(self.demo_target_pos)
                            self.demo_state = "PAUSE"
                            self.demo_state_start_time = now
                        else:
                            ratio = max(0.0, min(1.0, elapsed / self.link_duration))
                            s = 0.5 * (1.0 - math.cos(math.pi * ratio))
                            for i in range(5):
                                p_start = self.demo_start_pos[i]
                                p_target = self.demo_target_pos[i]
                                self.current_positions[i] = p_start + s * (p_target - p_start)

                    elif self.demo_state == "PAUSE":
                        elapsed = now - self.demo_state_start_time
                        step_pause = 2.0 if self.demo_step_idx == 4 else self.pause_duration
                        if elapsed >= step_pause:
                            if not self.demo_loop and self.demo_step_idx == 4:
                                # Completed 1-way deployment up to Link 5! Hold pose.
                                self.demo_enabled = False
                                self.get_logger().info('Hoan thanh chay 1 chieu Link 1 -> Link 5! Giu nguyen tu the.')
                            else:
                                self.demo_step_idx = (self.demo_step_idx + 1) % len(DEMO_STEPS)
                                self.start_next_demo_step()

                # Publish JointState using ROS clock for strictly monotonic stamps
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = list(self.current_positions)
                self.js_pub.publish(msg)

            t_calc = time.monotonic() - t_start
            sleep_time = period - t_calc
            if sleep_time > 0:
                time.sleep(sleep_time)

def main(args=None):
    rclpy.init(args=args)
    node = DensoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
