# -*- coding: utf-8 -*-
import sys, os, math, time, argparse
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'src', 'denso_vs6556_moveit_config', 'scripts'))
try:
    from denso_dynamics_engine import DensoDynamicsEngine, JOINT_LIMITS, JOINT_NAMES
except ImportError:
    sys.path.append('/home/dung/denso_ws/src/denso_vs6556_moveit_config/scripts')
    from denso_dynamics_engine import DensoDynamicsEngine, JOINT_LIMITS, JOINT_NAMES

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class DensoCartesianController(Node):
    def __init__(self):
        super().__init__('denso_cartesian_controller')
        self.engine = DensoDynamicsEngine()
        self.current_joints = np.zeros(5)
        self.received_js = False

        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_callback, 10
        )
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.get_logger().info('Denso Cartesian Controller khoi tao thanh cong.')

    def _js_callback(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(JOINT_NAMES)}
        for name, pos in zip(msg.name, msg.position):
            if name in name_to_idx:
                self.current_joints[name_to_idx[name]] = pos
        self.received_js = True

    def get_current_cartesian(self, use_flange=False):
        transforms, _ = self.engine.get_dh_transforms(self.current_joints)
        idx = 5 if use_flange else 4
        return transforms[idx][:3, 3] * 1000.0

    def solve_ik(self, target_xyz_mm, use_flange=False):
        target_m = np.array(target_xyz_mm) / 1000.0
        q_sol, success, iters, err_m = self.engine.inverse_kinematics(
            target_m, q_seed=self.current_joints, use_flange=use_flange
        )
        return q_sol, success, iters, err_m * 1000.0

    def move_to_joints(self, target_rad, duration_sec=2.0):
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in target_rad]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points = [pt]
        self.traj_pub.publish(traj)

def print_table(current_q, target_q, title='BANG GOC CAC JOINT CUA ROBOT'):
    JOINT_DISPLAY_NAMES = [
        'J1 De (Yaw - Z)',
        'J2 Vai (Pitch - Y)',
        'J3 Khuyu (Pitch - Y)',
        'J4 Cang tay (Roll - Z)',
        'J5 Co tay (Pitch - Y)'
    ]
    sep = '=' * 90
    print('')
    print(sep)
    print(f' {title:^88}')
    print(sep)
    print(f'{'Khop (Joint)':<22} | {'Hien Tai (deg)':<14} | {'Hien Tai (rad)':<15} | {'Muc Tieu (deg)':<14} | {'Do Lech Dq (deg)':<15}')
    print('-' * 90)
    for i in range(5):
        cur_deg = math.degrees(current_q[i])
        cur_rad = current_q[i]
        tar_deg = math.degrees(target_q[i])
        diff_deg = abs(tar_deg - cur_deg)
        print(f'{JOINT_DISPLAY_NAMES[i]:<22} | {cur_deg:+10.2f} deg  | {cur_rad:+12.4f} rad  | {tar_deg:+10.2f} deg  | {diff_deg:11.2f} deg')
    print(sep)

def main():
    parser = argparse.ArgumentParser(description='Dieu khien vi tri Link 5 Robot Denso VS-6556')
    parser.add_argument('--x', type=float, default=None, help='Toa do X muc tieu (mm)')
    parser.add_argument('--y', type=float, default=None, help='Toa do Y muc tieu (mm)')
    parser.add_argument('--z', type=float, default=None, help='Toa do Z muc tieu (mm)')
    parser.add_argument('--time', type=float, default=2.5, help='Thoi gian di chuyen (giay, mac dinh 2.5s)')
    parser.add_argument('--flange', action='store_true', help='Tinh theo Mat bich Flange thay vi Point P')
    parser.add_argument('-i', '--interactive', action='store_true', help='Che do nhap tu ban phim')

    args = parser.parse_args()

    rclpy.init()
    node = DensoCartesianController()

    print('Dang ket noi toi topic /joint_states cua Robot...')
    start_wait = time.time()
    while rclpy.ok() and not node.received_js and (time.time() - start_wait < 3.0):
        rclpy.spin_once(node, timeout_sec=0.1)

    cur_pos = node.get_current_cartesian(use_flange=args.flange)
    target_name = 'Flange End-Effector (+80mm)' if args.flange else 'Link 5 / Point P (Tam co tay)'
    print(f'Da ket noi! Doi tuong muc tieu: {target_name}')
    print(f'Toa do hien tai cua {target_name}: X = {cur_pos[0]:+.1f} mm, Y = {cur_pos[1]:+.1f} mm, Z = {cur_pos[2]:+.1f} mm')

    tx, ty, tz = args.x, args.y, args.z
    if args.interactive or (tx is None or ty is None or tz is None):
        print('')
        print('-' * 60)
        print('  BANG NHAP VI TRI MUC TIEU CUA LINK 5')
        print('-' * 60)
        print('Goi y cac vi tri an toan:')
        print('  1. Home:             X =   15.0 mm, Y =   0.0 mm, Z = 900.0 mm')
        print('  2. Phia truoc:       X =  350.0 mm, Y =   0.0 mm, Z = 600.0 mm')
        print('  3. Phia truoc - lech: X =  300.0 mm, Y = 150.0 mm, Z = 550.0 mm')
        print('-' * 60)
        try:
            inp_x = input(f'Nhap toa do X (mm) [Mac dinh {cur_pos[0]:.1f}]: ').strip()
            tx = float(inp_x) if inp_x else cur_pos[0]

            inp_y = input(f'Nhap toa do Y (mm) [Mac dinh {cur_pos[1]:.1f}]: ').strip()
            ty = float(inp_y) if inp_y else cur_pos[1]

            inp_z = input(f'Nhap toa do Z (mm) [Mac dinh {cur_pos[2]:.1f}]: ').strip()
            tz = float(inp_z) if inp_z else cur_pos[2]

            inp_t = input(f'Nhap thoi gian di chuyen (s) [Mac dinh {args.time:.1f}]: ').strip()
            duration = float(inp_t) if inp_t else args.time
        except ValueError:
            print('Du lieu nhap khong hop le!')
            node.destroy_node()
            rclpy.shutdown()
            return
    else:
        duration = args.time

    target_pos = [tx, ty, tz]
    print(f'Dang giai dong hoc nghich (IK) toi muc tieu: [{tx:+.1f}, {ty:+.1f}, {tz:+.1f}] mm...')

    q_sol, success, iters, err_mm = node.solve_ik(target_pos, use_flange=args.flange)

    if not success:
        print(f'CANH BAO: Vi tri nam ngoai tam voi an toan! Sai so nho nhat: {err_mm:.2f} mm')
        ans = input('Ban co muon tiep tuc gui vi tri gan nhat khong? (y/N): ').strip().lower()
        if ans != 'y':
            print('Da huy lenh.')
            node.destroy_node()
            rclpy.shutdown()
            return
    else:
        print(f'Giai IK thanh cong sau {iters} buoc lap! Sai so hinh hoc: {err_mm:.4f} mm')

    print_table(node.current_joints, q_sol, title='BANG GOC CAC JOINT CAN DAT TOI (IK SOLUTION)')

    print(f'Dang gui lenh cho robot chuyen dong trong {duration:.1f} giay...')
    node.move_to_joints(q_sol, duration_sec=duration)

    t_start = time.time()
    while rclpy.ok() and (time.time() - t_start < duration + 1.0):
        rclpy.spin_once(node, timeout_sec=0.1)
        cur_cart = node.get_current_cartesian(use_flange=args.flange)
        dist_remain = np.linalg.norm(np.array(target_pos) - cur_cart)
        time_elapsed = time.time() - t_start
        pct = min(100.0, (time_elapsed / duration) * 100.0)
        sys.stdout.write(f'\rTien trinh: {pct:5.1f}% | Khoang cach toi dich: {dist_remain:6.1f} mm | Toa do: [{cur_cart[0]:+6.1f}, {cur_cart[1]:+6.1f}, {cur_cart[2]:+6.1f}] mm')
        sys.stdout.flush()
        time.sleep(0.08)

    print('')
    print('Robot da hoan thanh chuyen dong!')
    final_cart = node.get_current_cartesian(use_flange=args.flange)
    print(f'Toa do thuc te cuoi cung: X = {final_cart[0]:+.2f} mm, Y = {final_cart[1]:+.2f} mm, Z = {final_cart[2]:+.2f} mm')
    print_table(node.current_joints, q_sol, title='BANG GOC CAC JOINT THUC TE KHI TOI DICH')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
