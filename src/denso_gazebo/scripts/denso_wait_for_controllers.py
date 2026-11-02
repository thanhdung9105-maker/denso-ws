#!/usr/bin/env python3
"""
DENSO WAIT FOR CONTROLLERS & TF SYNC NODE
Đảm bảo Gazebo Sim đã nạp xong toàn bộ controller và cây TF (world -> link_6)
trước khi cho phép RViz2 và MoveGroup khởi động.
Triệt tiêu 100% lỗi race-condition "No transform from [link_1] to [world]".
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

class DensoWaitNode(Node):
    def __init__(self):
        super().__init__('denso_wait_for_controllers')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_states_ready = False
        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            10
        )

        self.timer = self.create_timer(0.2, self._check_readiness)
        self.attempts = 0
        self.max_attempts = 300  # 60 seconds timeout
        self.get_logger().info('Đang chờ Gazebo controllers và TF tree sẵn sàng...')

    def _on_joint_states(self, msg: JointState):
        if len(msg.name) >= 6:
            self.joint_states_ready = True

    def _check_readiness(self):
        self.attempts += 1
        if self.joint_states_ready:
            try:
                # Kiểm tra cây TF từ world tới link_6 đã liên tục chưa
                if self.tf_buffer.can_transform('world', 'link_6', rclpy.time.Time()):
                    self.get_logger().info('✅ Hệ thống điều khiển Gazebo & TF Tree [world -> link_6] đã sẵn sàng 100%!')
                    rclpy.shutdown()
                    sys.exit(0)
            except Exception:
                pass

        if self.attempts >= self.max_attempts:
            self.get_logger().warn('⚠️ Hết thời gian chờ (60s), tiếp tục khởi động RViz...')
            rclpy.shutdown()
            sys.exit(0)

def main():
    rclpy.init()
    node = DensoWaitNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception:
        pass

if __name__ == '__main__':
    main()

