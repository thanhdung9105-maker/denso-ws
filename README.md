# Denso VS-6556 Industrial Robot Arm - ROS 2 Jazzy & MoveIt 2

Hệ thống điều khiển, mô phỏng và tính toán động học cho cánh tay robot công nghiệp **Denso VS-6556** (5 bậc tự do / 5-DOF) chạy trên nền tảng **ROS 2 Jazzy** và **MoveIt 2**, hỗ trợ chạy Native trong WSL2 (Ubuntu 24.04) hoặc cô lập hoàn toàn qua **Docker container** kèm đồ họa WSLg.

---

## 🌟 Tính Năng Nổi Bật

- **Mô hình hóa Robot chuẩn xác (URDF)**: Hiệu chỉnh chính xác gốc tọa độ và khớp xoay theo bản vẽ đo đạc thực tế từ SolidWorks.
- **Tích hợp MoveIt 2**: Cấu hình quy hoạch quỹ đạo OMPL và Pilz Industrial Motion Planner với thuật toán KDL Kinematics.
- **Bảng Điều Khiển Động Học Thời Gian Thực (Kinematics Dashboard)**:
  - **Động học thuận (FK)**: Hiển thị bảng góc 5 khớp (Rad/Độ) và vị trí/hướng 3D của đầu gắp ($X, Y, Z$, Roll, Pitch, Yaw) cập nhật tức thời từ `/joint_states`.
  - **Động học nghịch (IK)**: Bộ giải Damped Least Squares (Levenberg-Marquardt) tốc độ cao ($<2\text{ms}$, sai số $<0.5\text{mm}$), hỗ trợ gửi lệnh trực tiếp cho robot di chuyển trong RViz2.
- **Khởi động an toàn**: Mặc định cố định toàn bộ 5 khớp tại vị trí gốc $0^\circ$, chống rung lắc hoặc chạy mất kiểm soát khi vừa khởi động.
- **Đóng gói Docker trọn gói**: Chạy độc lập với `docker compose`, chuyển tiếp đồ họa X11/Wayland mượt mà lên Windows.

---

## 🚀 Hướng Dẫn Cài Đặt & Khởi Chạy

### 1. Khởi chạy trên máy chủ (Native ROS 2)

```bash
# Bật cánh tay robot và giao diện RViz2 + MoveIt 2
denso

# Mở Bảng Điều Khiển Động Học Thuận & Nghịch (FK & IK)
denso_gui

# Điều khiển chuyển động tuần tự các khớp
denso_joints  # Chạy tuần hoàn Link 1 -> Link 5 (2.0s / link)
denso_once    # Chạy 1 lượt rồi giữ nguyên tư thế
denso_home    # Trở về tư thế thẳng đứng 0°
denso_stop    # Dừng khẩn cấp / Giữ vị trí
```

### 2. Khởi chạy trong Docker

```bash
# Khởi chạy robot và RViz2 trong Docker container
denso_docker

# Mở bash terminal bên trong container ROS 2
ros_docker
```

---

## 📁 Cấu Trúc Workspace

```
denso_ws/
├── src/
│   ├── denso_vs6556/                # Gói mô tả robot (URDF, STL Meshes, RViz config)
│   └── denso_vs6556_moveit_config/  # Cấu hình MoveIt 2, controller, GUI động học
├── docker/                          # Dockerfile, docker-compose.yml, run scripts
├── launch_denso.sh                  # Script khởi chạy nhanh hệ thống
├── denso_joints.sh                  # Script gửi lệnh demo khớp
├── denso_gui.sh                     # Script mở Bảng Động Học (GUI)
├── .gitignore
└── README.md
```

---

## 👤 Tác Giả
- **Tác giả**: thanhdung88 (thanhdung9105@gmail.com)
- **Hệ điều hành**: Ubuntu 24.04 LTS (WSL2) / Windows 11
- **Phiên bản ROS**: ROS 2 Jazzy Jalisco
