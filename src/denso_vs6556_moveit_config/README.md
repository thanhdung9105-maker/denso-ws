# Denso VS-6556 — MoveIt 2 Config & Dynamics Dashboard

## Kiến Trúc Node Graph

```
/joint_states ←──────────────────────────────────────────────────────┐
/denso/status ←────────────────────────────────────────────────────┐  │
/denso/joint_dynamics_markers ←─────────────────────────────────┐  │  │
                                                                  │  │  │
┌─────────────────────────┐    ┌──────────────────────────────┐  │  │  │
│  denso_mock_controller  │───▶│       robot_state_publisher  │──┘  │  │
│  • RNEA Dynamics        │    │  (URDF TF tree publisher)    │     │  │
│  • FollowJointTraj AS   │────────────────────────────────────▶    │  │
│  • /denso/cmd (sub)     │                                         │  │
│  • /denso/status (pub)  │─────────────────────────────────────────▶  │
│  • 10Hz control loop    │─────────────────────────────────────────────▶
└─────────────────────────┘
         ▲  ▲
         │  │  /arm_controller/follow_joint_trajectory (action)
         │  └────────── MoveGroup (optional, demo.launch.py)
         │
         └────────────── /denso/cmd (String)
                         denso_kinematics_gui.py
                         denso_joint_by_joint_demo.py
```

## Cách Chạy

### 1. Standalone (không cần MoveIt)
```bash
# Source workspace
source ~/denso_ws/install/setup.bash

# Chạy toàn bộ stack + GUI (mặc định)
ros2 launch denso_vs6556_moveit_config kinematics_gui.launch.py

# Thêm RViz2
ros2 launch denso_vs6556_moveit_config kinematics_gui.launch.py launch_rviz:=true

# Chỉ controller backend (không GUI)
ros2 launch denso_vs6556_moveit_config kinematics_gui.launch.py launch_gui:=false
```

### 2. MoveIt Full Stack
```bash
# MoveIt + RViz2 (mặc định)
ros2 launch denso_vs6556_moveit_config demo.launch.py

# MoveIt + RViz2 + GUI Dashboard
ros2 launch denso_vs6556_moveit_config demo.launch.py launch_gui:=true

# MoveIt không RViz2
ros2 launch denso_vs6556_moveit_config demo.launch.py launch_rviz:=false
```

### 3. Demo Tự Động
```bash
# Chạy demo Link 1→5 lặp lại
ros2 run denso_vs6556_moveit_config denso_joint_by_joint_demo.py

# Về Home
ros2 run denso_vs6556_moveit_config denso_joint_by_joint_demo.py --home
```

## Topics

| Topic | Msg Type | Hướng | Mô Tả |
|-------|----------|-------|-------|
| `/joint_states` | `sensor_msgs/JointState` | Pub | Vị trí/vận tốc/effort 10Hz |
| `/denso/cmd` | `std_msgs/String` | Sub | Lệnh: demo, stop, home, once, goto, payload |
| `/denso/status` | `std_msgs/String` | Pub | JSON trạng thái controller 2Hz |
| `/denso/joint_dynamics_markers` | `visualization_msgs/MarkerArray` | Pub | Vectors lực RViz2 |
| `/arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` Action | Server | MoveIt trajectory |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Sub | Direct trajectory |

## Bảng D-H Chuẩn Đã Xác Thực (Standard D-H, Denso VS-6556)

Bảng tham số Denavit-Hartenberg tiêu chuẩn được xác thực khớp tuyệt đối ($\Delta = 0.0000	ext{ mm}$) với URDF và mô hình lượng giác phẳng Sin-Cos:

| Khớp $ | Liên kết | $ (m) | $ (m) | $lpha_i$ (°) | Biến khớp $	heta_i$ (rad) | Đặc điểm hình học |
|:---:|---|:---:|:---:|:---:|:---:|---|
| **J1** | Base $	o$ Shoulder J2 | 0.335 | -0.075 | -90.0° | $	heta_1 = q_1$ | Trục $ thẳng đứng, chuyển sang trục xoay vai $ nằm ngang |
| **J2** | Upper Arm (J2 $	o$ J3) | 0.000 | 0.270 | 0.0° | $	heta_2 = q_2 - 90^\circ$ | Chiều dài xương cánh tay trên 270mm, 2 trục song song |
| **J3** | Elbow Offset | 0.000 | 0.090 | +90.0° | $	heta_3 = q_3 + 90^\circ$ | Độ lệch khuỷu tay 90mm ra phía trước, đổi hướng sang cẳng tay |
| **J4** | Forearm Roll | 0.295 | 0.000 | -90.0° | $	heta_4 = -q_4$ | Chiều dài cẳng tay 295mm, trục xoay Roll quanh cẳng tay |
| **J5** | Tool Flange TCP | 0.000 | 0.080 | 0.0° | $	heta_5 = q_5 - 90^\circ$ | Tâm cổ tay Point P vươn ra mặt bích gá dụng cụ (Flange TCP) |

**Tầm với:** Point P = R653.42mm | Flange TCP = R733.42mm (P + 80mm offset)
**Script kiểm chứng:** Chạy  để đối chiếu sai số tự động.

## Cấu Trúc Files

```
denso_vs6556_moveit_config/
├── config/
│   ├── joint_limits.yaml          # Giới hạn tốc độ/gia tốc/moment
│   ├── moveit_controllers.yaml    # FollowJointTrajectory config
│   ├── kinematics.yaml            # KDL kinematics plugin
│   ├── ompl_planning.yaml         # OMPL planner settings
│   └── pilz_cartesian_limits.yaml
├── launch/
│   ├── demo.launch.py             # MoveIt full stack
│   └── kinematics_gui.launch.py   # Standalone GUI (không MoveIt)
├── scripts/
│   ├── denso_dynamics_engine.py   # RNEA động lực học engine
│   ├── denso_mock_controller.py   # ROS2 controller node
│   ├── denso_kinematics_gui.py    # Qt5 Dashboard GUI (5 tabs)
│   ├── denso_animation_demo.py    # MoveGroup sequence demo
│   └── denso_joint_by_joint_demo.py  # CLI demo controller
└── README.md                      # Tài liệu này
```

## Tài Liệu Tham Khảo

1. Denso Wave Inc. *VS-6556/6577 User's Manual*. Doc No. 410590-0080.
2. Kütük M.E., Daş M.T., Dülger L.C. (2017). *Forward and Inverse Kinematics Analysis of Denso Robot*. AzCIFToMM. DOI: dergipark.org.tr
3. Tran M.P., Ha T.B. et al. *Nonlinear Controller with Dynamic Compensation for Denso VS-6556*. JTE-HCMUTE. jte.edu.vn
4. HCMUT. *FL061 Lab Manual — Simulating Motions of DENSO Robot VS-6556*. studocu.vn