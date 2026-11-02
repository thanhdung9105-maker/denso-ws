#!/usr/bin/env python3
"""
DENSO VS-6556 LINK CENTER OF MASS & MASS VISUALIZER FOR RVIZ2
Thiet ke bieu tuong Trong tam chuan Ky thuat tuong phan cao:
- Tam CoM: Qua cau Xanh ngoc Neon (Electric Cyan #00FFFF)
- Khung ngam Hong tam 3D: 2 vong xich dao va kinh tuyen mau Trang sang bao quanh.
- He truc quan tinh 3D: Truc X (Do), Y (Xanh la), Z (Xanh duong) dai 42mm, day 3.5mm.
- Nhan chu 3D thong so khoi luong ro rang, noi bat.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
import math

LINK_COM_DATA = [
    {
        'link': 'base_link',
        'name': 'Base (Đế)',
        'mass': 8.0,
        'xyz': [0.0, 0.0, 0.09],
        'label_offset': [0.12, 0.0, 0.06],
    },
    {
        'link': 'link_1',
        'name': 'Link 1',
        'mass': 4.5,
        'xyz': [-0.04, 0.0, 0.08],
        'label_offset': [-0.12, 0.0, 0.05],
    },
    {
        'link': 'link_2',
        'name': 'Link 2 (Bắp tay)',
        'mass': 4.2,
        # Tọa độ khối tâm CAD chính xác trong lòng thân cánh tay trên
        'xyz': [0.0, -0.041, 0.120],
        'label_offset': [0.0, -0.11, 0.05],
    },
    {
        'link': 'link_3',
        'name': 'Link 3 (Khuỷu)',
        'mass': 3.1,
        # Tọa độ khối tâm CAD chính xác trong lòng thân khuỷu tay
        'xyz': [0.045, 0.075, 0.010],
        'label_offset': [0.06, 0.08, 0.05],
    },
    {
        'link': 'link_4',
        'name': 'Link 4 (Cánh tay)',
        'mass': 2.0,
        'xyz': [0.0, 0.0, 0.10],
        'label_offset': [0.0, 0.09, 0.04],
    },
    {
        'link': 'link_5',
        'name': 'Link 5 (Cổ tay)',
        'mass': 0.95,
        'xyz': [0.0, 0.0, 0.0163],
        'label_offset': [0.0, 0.08, 0.03],
    },
    {
        'link': 'link_6',
        'name': 'Link 6 (Nắp bích xoay)',
        'mass': 0.25,
        'xyz': [0.0, 0.0, 0.01126],
        'label_offset': [0.0, 0.08, 0.02],
    },
]

class DensoCoMVisualizer(Node):
    def __init__(self):
        super().__init__('denso_com_visualizer')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/denso/com_markers', qos)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Xóa các marker thừa ở link_5/tool (id 6 trước đây)
        self._clear_legacy_markers()
        
        # Duy trì phát chu kỳ 0.2s (5 Hz) khi TF đã sẵn sàng
        # Giúp RViz2 nhận biểu tượng trọng tâm tức thì, mượt mà khi di chuyển khớp và triệt tiêu tải CPU
        self.timer = self.create_timer(0.2, self.publish_markers)
        self.get_logger().info('Denso CoM Visualizer: Da bat dau phat bieu tuong Trong tam Chuan Xac 6 Link.')

    def _clear_legacy_markers(self):
        clear_array = MarkerArray()
        # Xóa triệt để các marker của mục 5 và các id cũ để RViz dọn sạch bộ nhớ
        namespaces = [
            "1. Điểm Trọng tâm (Xanh Neon)",
            "2. Khung ngắm Hồng tâm (Vòng Trắng)",
            "3. Hệ trục tọa độ 3D (X-Y-Z)",
            "4. Đường chỉ dẫn ra ngoài",
            "5. Nhãn Tên & Khối lượng (kg)"
        ]
        all_ids = [6, 56, 76, 106, 156, 206] + list(range(200, 215))
        for ns in namespaces:
            for extra_id in all_ids:
                del_m = Marker()
                del_m.header.frame_id = "base_link"
                del_m.ns = ns
                del_m.id = extra_id
                del_m.action = Marker.DELETE
                clear_array.markers.append(del_m)
        self.marker_pub.publish(clear_array)

    def publish_markers(self):
        # Đảm bảo TF tree đã có link_6 trước khi xuất bản marker để triệt tiêu 100% lỗi RViz 'Frame does not exist'
        try:
            if not self.tf_buffer.can_transform('world', 'link_6', rclpy.time.Time()):
                return
        except Exception:
            return

        marker_array = MarkerArray()
        zero_stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()

        for i, item in enumerate(LINK_COM_DATA):
            link_name = item['link']
            short_name = item['name']
            mass = item['mass']
            x, y, z = item['xyz']
            dx, dy, dz = item['label_offset']
            mass_str = f"{mass:.2f} kg" if mass < 1.0 else f"{mass:.1f} kg"
            part_ns = f"Part {i} - {short_name} ({mass_str})"

            lx = x + dx
            ly = y + dy
            lz = z + dz

            # 0. Điểm Trọng tâm theo từng Part riêng biệt (để bật/tắt từng Part trong RViz)
            part_dot = Marker()
            part_dot.header.stamp = zero_stamp
            part_dot.header.frame_id = link_name
            part_dot.ns = part_ns
            part_dot.id = 300 + i
            part_dot.type = Marker.SPHERE
            part_dot.action = Marker.ADD
            part_dot.pose.position.x = float(x)
            part_dot.pose.position.y = float(y)
            part_dot.pose.position.z = float(z)
            part_dot.pose.orientation.w = 1.0
            part_dot.scale.x = 0.026
            part_dot.scale.y = 0.026
            part_dot.scale.z = 0.026
            part_dot.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.95)
            marker_array.markers.append(part_dot)

            # 1. Tâm Trọng tâm: Quả cầu Xanh ngọc Neon (Electric Cyan)
            dot = Marker()
            dot.header.stamp = zero_stamp
            dot.header.frame_id = link_name
            dot.ns = "1. Điểm Trọng tâm (Xanh Neon)"
            dot.id = i
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = float(x)
            dot.pose.position.y = float(y)
            dot.pose.position.z = float(z)
            dot.pose.orientation.w = 1.0

            d = 0.024  # Đường kính 24mm rõ nét
            dot.scale.x = d
            dot.scale.y = d
            dot.scale.z = d
            dot.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) # Cyan #00FFFF
            marker_array.markers.append(dot)

            # 2. Khung ngắm Hồng tâm 3D (Target Reticle Rings)
            ring_xy = Marker()
            ring_xy.header.stamp = zero_stamp
            ring_xy.header.frame_id = link_name
            ring_xy.ns = "2. Khung ngắm Hồng tâm (Vòng Trắng)"
            ring_xy.id = 50 + i
            ring_xy.type = Marker.LINE_STRIP
            ring_xy.action = Marker.ADD
            ring_xy.scale.x = 0.0025
            ring_xy.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)

            r_ring = 0.018  # Bán kính 18mm
            for step in range(17):
                angle = step * (2.0 * math.pi / 16.0)
                p = Point(
                    x=float(x + r_ring * math.cos(angle)),
                    y=float(y + r_ring * math.sin(angle)),
                    z=float(z)
                )
                ring_xy.points.append(p)
            marker_array.markers.append(ring_xy)

            # Vòng kinh tuyến đứng (Mặt phẳng XZ)
            ring_xz = Marker()
            ring_xz.header.stamp = zero_stamp
            ring_xz.header.frame_id = link_name
            ring_xz.ns = "2. Khung ngắm Hồng tâm (Vòng Trắng)"
            ring_xz.id = 70 + i
            ring_xz.type = Marker.LINE_STRIP
            ring_xz.action = Marker.ADD
            ring_xz.scale.x = 0.002
            ring_xz.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.85)
            for step in range(17):
                angle = step * (2.0 * math.pi / 16.0)
                p = Point(
                    x=float(x + r_ring * math.cos(angle)),
                    y=float(y),
                    z=float(z + r_ring * math.sin(angle))
                )
                ring_xz.points.append(p)
            marker_array.markers.append(ring_xz)

            # 3. Hệ trục tọa độ 3D Quán tính (CoM Triad)
            axes = Marker()
            axes.header.stamp = zero_stamp
            axes.header.frame_id = link_name
            axes.ns = "3. Hệ trục tọa độ 3D (X-Y-Z)"
            axes.id = 100 + i
            axes.type = Marker.LINE_LIST
            axes.action = Marker.ADD
            axes.scale.x = 0.0035

            axis_len = 0.042
            p_center = Point(x=float(x), y=float(y), z=float(z))

            # Trục X: Đỏ
            p_x = Point(x=float(x + axis_len), y=float(y), z=float(z))
            c_red = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
            axes.points.extend([p_center, p_x])
            axes.colors.extend([c_red, c_red])

            # Trục Y: Xanh lá
            p_y = Point(x=float(x), y=float(y + axis_len), z=float(z))
            c_green = ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0)
            axes.points.extend([p_center, p_y])
            axes.colors.extend([c_green, c_green])

            # Trục Z: Xanh dương
            p_z = Point(x=float(x), y=float(y), z=float(z + axis_len))
            c_blue = ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0)
            axes.points.extend([p_center, p_z])
            axes.colors.extend([c_blue, c_blue])

            marker_array.markers.append(axes)

            # 4. Đường chỉ dẫn ra ngoài & Nhãn thông số khối lượng 3D
            leader = Marker()
            leader.header.stamp = zero_stamp
            leader.header.frame_id = link_name
            leader.ns = "4. Đường chỉ dẫn ra ngoài"
            leader.id = 150 + i
            leader.type = Marker.LINE_LIST
            leader.action = Marker.ADD
            leader.scale.x = 0.002
            p_com = Point(x=float(x), y=float(y), z=float(z))
            p_lbl = Point(x=float(lx), y=float(ly), z=float(lz))
            c_lead = ColorRGBA(r=0.0, g=0.95, b=1.0, a=0.85)
            leader.points.extend([p_com, p_lbl])
            leader.colors.extend([c_lead, c_lead])
            marker_array.markers.append(leader)

            text_mk = Marker()
            text_mk.header.stamp = zero_stamp
            text_mk.header.frame_id = link_name
            text_mk.ns = "5. Nhãn khối lượng & Trọng tâm"
            text_mk.id = 200 + i
            text_mk.type = Marker.TEXT_VIEW_FACING
            text_mk.action = Marker.ADD
            text_mk.pose.position.x = float(lx)
            text_mk.pose.position.y = float(ly)
            text_mk.pose.position.z = float(lz + 0.015)
            text_mk.pose.orientation.w = 1.0
            text_mk.scale.z = 0.024
            text_mk.color = ColorRGBA(r=1.0, g=0.95, b=0.4, a=0.95)
            text_mk.text = f"{short_name} ({mass:.2f}kg)" if mass < 1.0 else f"{short_name} ({mass:.1f}kg)"
            marker_array.markers.append(text_mk)



        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DensoCoMVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
