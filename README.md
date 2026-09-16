# Denso VS-6556 Industrial Robot Arm - ROS 2 Jazzy & MoveIt 2

Hệ thống điều khiển, mô phỏng 3D và tính toán động học cho cánh tay robot công nghiệp **Denso VS-6556** (5 bậc tự do / 5-DOF) chạy trên nền tảng **ROS 2 Jazzy** và **MoveIt 2**, hỗ trợ chạy Native trong WSL2 (Ubuntu 24.04) hoặc cô lập qua **Docker container** kèm đồ họa WSLg.

Repository này được liên kết chính thức với GitHub:  
👉 **[https://github.com/thanhdung9105-maker/denso-ws](https://github.com/thanhdung9105-maker/denso-ws)**

---

## 🏗️ 1. Kiến Trúc Lưu Trữ & Hiệu Năng: Ổ D: vs WSL2

### ❓ Câu hỏi thường gặp: "Nên clone code vào WSL hay clone vào ổ D: để chạy?"

| Tiêu chí | Lưu tại ổ `D:/ROS2/denso-ws` (Windows NTFS) | Lưu tại `/home/dung/denso_ws` (WSL2 ext4) |
|---|---|---|
| **Mục đích sử dụng** | Quản lý file, chỉnh sửa code bằng VS Code Windows, commit/push Git qua Git GUI. | Build mã nguồn (`colcon build`), chạy node ROS 2, RViz2, MoveIt 2. |
| **Tốc độ đọc/ghi I/O** | Chậm hơn (do đi qua cầu nối ảo 9P giữa Linux và Windows NTFS). | **Nhanh gấp 5 – 10 lần** (chạy trực tiếp trên hệ thống file ext4 gốc của Linux). |
| **Phân quyền Linux & Symlink** | Hạn chế (NTFS không hỗ trợ chuẩn `chmod +x` và symlink của ROS 2). | **Hỗ trợ 100%** chuẩn POSIX Linux, không bao giờ bị lỗi permission. |

### 💡 Quy Trình Làm Việc Chuẩn (Recommended Dual-Workflow):
1. **Chỉnh sửa & Quản lý Git trên Windows (`D:/ROS2/denso-ws`)**:
   - Mở thư mục `D:\ROS2\denso-ws` trong VS Code hoặc Antigravity IDE trên Windows để viết code, xem tài liệu, quản lý commit.
   - Dùng Git trên Windows để push code lên GitHub cực kỳ dễ dàng (tự động nhận Git Credential Manager của Windows, không bị hỏi mật khẩu).
2. **Chạy & Build trong WSL2 (`/home/dung/denso_ws`)**:
   - Thư mục `/home/dung/denso_ws` trong WSL đóng vai trò là "Runtime Engine" để biên dịch và chạy robot ở tốc độ tối đa.
3. **Đồng bộ giữa D: và WSL2**:
   - Khi sửa code trên ổ D: và muốn đồng bộ sang WSL:
     ```bash
     # Trong terminal WSL:
     cd /home/dung/denso_ws
     git pull /mnt/d/ROS2/denso-ws main
     colcon build --symlink-install
     ```
   - Hoặc đơn giản là push từ D: lên GitHub, rồi trong WSL gõ `git pull origin main`.

---

## 🚀 2. Hướng Dẫn Khởi Chạy Hệ Thống

### Cách 1: Khởi chạy bằng Docker (Không lo xung đột môi trường)

Docker image `denso_ros2_jazzy:latest` đã được build sẵn, tích hợp đầy đủ ROS 2 Jazzy, MoveIt 2, CycloneDDS và thư viện đồ họa tối ưu cho WSLg:

```bash
# Khởi chạy robot và RViz2 trong Docker container (tự mở cửa sổ trên Windows)
denso_docker

# Mở một terminal Bash tương tác bên trong container ROS 2
ros_docker
```

---

### Cách 2: Khởi chạy Native trong WSL2 (Hiệu năng cao nhất)

#### Bước 1: Build workspace (nếu có thay đổi code)
```bash
cd /home/dung/denso_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

#### Bước 2: Khởi chạy mô phỏng Robot
Mở Terminal và gõ:
```bash
denso
```
*(Cửa sổ RViz2 sẽ hiện lên. Cánh tay robot mặc định đứng yên cố định ở vị trí gốc $0^\circ$ an toàn).*

#### Bước 3: Mở Bảng Điều Khiển Động Học Thuận & Nghịch (Kinematics Dashboard)
Mở một Terminal khác và gõ:
```bash
denso_gui
```
*(Cửa sổ GUI sẽ mở lên, cho phép xem trực tiếp tọa độ $X, Y, Z$, Roll, Pitch, Yaw của đầu gắp, nhập tọa độ gắp vật để giải IK và bấm gửi lệnh cho robot di chuyển).*

---

## 🎮 3. Danh Sách Lệnh Điều Khiển Nhanh (Aliases)

Các lệnh này đã được cấu hình sẵn trong `~/.bashrc`:

| Lệnh | Chức năng |
|---|---|
| `denso` | Bật toàn bộ hệ thống Robot + MoveIt 2 + RViz2 trên máy thật. |
| `denso_docker` | Bật toàn bộ hệ thống Robot bên trong Docker container độc lập. |
| `denso_gui` | Mở Bảng Điều Khiển Động Học Thuận (FK) & Nghịch (IK). |
| `denso_joints` | Kích hoạt robot chạy chu trình khớp 1 chiều liên tục (Link 1 $\to$ Link 5). |
| `denso_once` | Chạy đúng 1 lượt từ Link 1 $\to$ Link 5 (2.0s/link) rồi dừng hẳn ở vị trí đích. |
| `denso_home` | Đưa cánh tay robot trở về tư thế đứng thẳng $0^\circ$. |
| `denso_stop` | Tạm dừng chuyển động ngay lập tức tại vị trí hiện tại. |
| `ros_docker` | Mở terminal bash bên trong Docker container ROS 2 Jazzy. |

---

## 📊 4. Tính Năng Bảng Động Học & Động Lực Học (Kinematics & Dynamics Dashboard)

* **1. Động Học Thuận (Forward Kinematics - FK)**:
  - Lắng nghe real-time topic `/joint_states`.
  - Hiển thị góc của 5 trục: $J_1$ (Base Z), $J_2$ (Shoulder Y), $J_3$ (Elbow Y), $J_4$ (Forearm X), $J_5$ (Wrist Y) theo cả **Radian** và **Độ ($^\circ$)**.
  - Tính toán ma trận biến đổi thuần nhất $T_{0}^{5}$ để xuất tọa độ đầu gắp: $X, Y, Z$ (mm & m), Roll, Pitch, Yaw ($^\circ$), bán kính với gốc $R$.
  - Thanh trượt Jogging cho phép kéo thử từng góc khớp để xem tọa độ thay đổi tức thời.
* **2. Động Học Nghịch (Inverse Kinematics - IK)**:
  - Ô nhập tọa độ mục tiêu $X, Y, Z$ (mm).
  - Nút *"📌 Lấy tọa độ hiện tại"* giúp lấy nhanh vị trí làm mốc.
  - Bộ giải Jacobian Damped Least Squares (Levenberg-Marquardt) tốc độ $<2\text{ms}$, sai số $<0.5\text{mm}$.
  - Nút *"🚀 Gửi tới Robot trong RViz2"* để điều khiển cánh tay xoay mượt mà theo đường cong S-curve tới tọa độ mục tiêu.
* **3. Động Lực Học Nghịch (Inverse Dynamics - ID)**:
  - Thuật toán đệ quy **RNEA (Recursive Newton-Euler Algorithm)** tính toán mô-men xoắn yêu cầu $\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) + \tau_f + J^T F_{ext}$.
  - Phân tích chi tiết từng thành phần: Lực quán tính $M\ddot{q}$, Lực Coriolis & ly tâm $C\dot{q}$, Trọng lực $g(q)$, Lực do tải trọng $J^TF$, Ma sát $\tau_f$.
  - Nút *"⚖️ Cân bằng trọng lực tĩnh"* để tính nhanh mô-men giữ cánh tay chống rơi tự do.
  - Hỗ trợ nhập tải trọng đầu gắp $m_{payload}$ (0 - 5kg) và ngoại lực $F_z$ (N).
  - Thanh đo % tải trọng động cơ kèm mã màu cảnh báo (Xanh an toàn, Vàng cảnh báo, Đỏ quá tải).
* **4. Động Lực Học Thuận (Forward Dynamics - FD)**:
  - Giải bài toán gia tốc góc khớp: $\ddot{q} = M(q)^{-1} (\tau - C(q, \dot{q})\dot{q} - g(q) - \tau_f - J^T F_{ext})$.
  - Thanh trượt chỉnh mô-men xoắn $\tau_1 \dots \tau_5$ (N·m) cho từng khớp.
  - Xuất ma trận khối lượng đối xứng xác định dương $M(q)$ kích thước $5 \times 5$ và vector gia tốc góc $\ddot{q}$ ($rad/s^2$ & $deg/s^2$).
  - **Mô phỏng tương tác vật lý thời gian thực trên RViz2 (20Hz)**: Bấm *"▶️ Bắt đầu mô phỏng RViz2"*, người dùng kéo thanh trượt mô-men xoắn, cánh tay robot trong RViz2 sẽ chuyển động và gia tốc theo đúng quy luật lực Newton!
* **5. Trực Quan Hóa Động Lực Học 3D Trên RViz2**:
  - Tự động hiển thị tại 5 khớp của robot trong RViz2 qua topic `/denso/joint_dynamics_markers`:
    + **Mũi tên vector mô-men xoắn 3D (Torque Arrows)**: Căn chỉnh đúng trục quay từng khớp, tự co giãn theo độ lớn và đổi màu cảnh báo tải trọng.
    + **Nhãn chữ nổi 3D (3D Billboard Text Labels)**: Luôn hướng về phía camera, hiển thị chính xác tên khớp, giá trị Torque (N·m), % tải và gia tốc $\ddot{q}$.

---

## 📁 5. Cấu Trúc Thư Mục Repository

```
denso_ws/
├── src/
│   ├── denso_vs6556/                # Gói mô tả robot
│   │   ├── urdf/                    # denso_vs6556.urdf (đã căn chỉnh chuẩn SolidWorks)
│   │   ├── meshes/                  # File 3D CAD STL của các link
│   │   ├── launch/                  # display.launch.py
│   │   └── rviz/                    # display.rviz (cấu hình giao diện RViz)
│   │
│   └── denso_vs6556_moveit_config/  # Gói cấu hình MoveIt 2 & điều khiển
│       ├── config/                  # kinematics.yaml, joint_limits.yaml, srdf
│       ├── scripts/
│       │   ├── denso_kinematics_gui.py  # Mã nguồn Bảng điều khiển Động học FK/IK
│       │   └── denso_mock_controller.py # Bộ điều khiển nội suy khớp S-curve 10Hz
│       └── launch/
│           └── demo.launch.py
│
├── docker/                          # Môi trường container hóa
│   ├── Dockerfile                   # Base image ros:jazzy + MoveIt 2 + CycloneDDS
│   ├── docker-compose.yml           # Cấu hình map volume và GUI WSLg
│   ├── entrypoint.sh                # Auto source ROS 2
│   └── run_denso_docker.sh          # Script chạy container 1 chạm
│
├── launch_denso.sh                  # Script chạy nhanh hệ thống
├── denso_joints.sh                  # Script demo chu trình khớp
├── denso_gui.sh                     # Script bật Bảng Động Học
├── .gitignore                       # Bỏ qua build, install, log
└── README.md                        # Tài liệu hướng dẫn
```

---

## 🔄 6. Hướng Dẫn Push Lên GitHub

Vì thư mục `D:\ROS2\denso-ws` nằm trực tiếp trên Windows, bạn có thể đẩy code lên GitHub theo một trong các cách cực kỳ tiện lợi sau:

### Cách 1: Sử dụng Terminal Windows (PowerShell / CMD)
Mở PowerShell tại thư mục `D:\ROS2\denso-ws`:
```powershell
cd D:\ROS2\denso-ws
git push -u origin main
```
*(Windows sẽ tự động mở hộp thoại đăng nhập GitHub thông qua Git Credential Manager mà không cần tạo token thủ công!)*

### Cách 2: Sử dụng VS Code trên Windows
1. Mở thư mục `D:\ROS2\denso-ws` bằng VS Code.
2. Vào tab **Source Control** (`Ctrl + Shift + G`).
3. Bấm **Sync Changes** hoặc **Push** $\to$ VS Code sẽ tự động đồng bộ lên GitHub.

---

## 👤 Tác Giả & Bản Quyền
- **Tác giả**: thanhdung88 (thanhdung9105@gmail.com)
- **GitHub**: [thanhdung9105-maker](https://github.com/thanhdung9105-maker)
- **Robot**: Denso VS-6556 Articulated Industrial Robot
