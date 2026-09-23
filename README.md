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

### Cách 3: Khởi chạy Mô Phỏng Vật Lý Gazebo Harmonic (`gz-sim 8`)

Hệ thống tích hợp gói mô phỏng vật lý `denso_gazebo` với phần cứng mô phỏng `gz_ros2_control`:

#### 1. Khởi chạy Gazebo Simulation độc lập:
```bash
denso_gazebo
```
*(Cửa sổ Gazebo Harmonic sẽ mở ra cùng cánh tay robot Denso VS-6556 đặt trên sàn mô phỏng. Trình điều khiển `arm_controller` và `joint_state_broadcaster` tự động kích hoạt).*

#### 2. Khởi chạy Gazebo Sim kết hợp MoveIt 2 & RViz2:
```bash
denso_gazebo_moveit
```
*(Mở đồng thời cả Gazebo Sim và RViz2. Mọi trajectory được MoveIt lập kế hoạch sẽ được gửi trực tiếp và thực thi chính xác trên thế giới vật lý ảo).*

#### 3. Kiểm thử tự động quỹ đạo 5 khớp:
Mở một terminal khác và gõ:
```bash
denso_gazebo_test
```
*(Robot sẽ tự động duyệt qua các điểm mốc tư thế làm việc và trở về Home, xác nhận hệ thống chạy ổn định 100%).*

---

## 🎮 3. Danh Sách Lệnh Điều Khiển Nhanh (Aliases)

Các lệnh này đã được cấu hình sẵn trong `~/.bashrc`:

| Lệnh | Chức năng |
|---|---|
| `denso` | Bật toàn bộ hệ thống Robot + MoveIt 2 + RViz2 trên máy thật. |
| `denso_gazebo` | **Bật mô phỏng vật lý Gazebo Harmonic (gz-sim 8)** với robot Denso VS-6556. |
| `denso_gazebo_moveit` | **Bật đồng thời Gazebo + MoveIt 2 + RViz2** (đồng bộ điều khiển thời gian thực). |
| `denso_gazebo_test` | Chạy kịch bản tự động kiểm thử quỹ đạo 5 khớp chuyển động ổn định trên Gazebo. |
| `denso_docker` | Bật toàn bộ hệ thống Robot bên trong Docker container độc lập. |
| `denso_table` / `denso_gui` | Mở **Bảng Thông Số Động Lực Học (Nền Trắng)** và Giám Sát Thời Gian Thực. |
| `denso_dynamics` | Bật / Tắt (Ẩn / Hiện) tức thì các vector mô-men xoắn trong RViz2. |
| `denso_joints` | Kích hoạt robot chạy chu trình khớp 1 chiều liên tục (Link 1 $\to$ Link 5). |
| `denso_once` | Chạy đúng 1 lượt từ Link 1 $\to$ Link 5 (2.0s/link) rồi dừng hẳn ở vị trí đích. |
| `denso_home` | Đưa cánh tay robot trở về tư thế đứng thẳng $0^\circ$. |
| `denso_stop` | Tạm dừng chuyển động ngay lập tức tại vị trí hiện tại. |
| `ros_docker` | Mở terminal bash bên trong Docker container ROS 2 Jazzy. |

---

## 📊 4. Tính Năng Bảng Giám Sát Động Học & Động Lực Học (Dashboard)

Hệ thống được thiết kế tối ưu, tách biệt thành các tab chuyên sâu, kết hợp toàn diện cả **Động Lực Học (Dynamics)** lẫn **Động Học D-H (Kinematics)** và tọa độ không gian làm việc của đầu gắp:

* **Tab 1: Bảng Thông Số Động Lực Học (Nền Trắng - Real-time Telemetry)**:
  - **Giao diện Nền Trắng Siêu Rõ Nét (`#ffffff`)**: Phông chữ đen đậm, viền chuẩn, tương phản cao, dễ quan sát từ xa như màn hình giám sát công nghiệp tiêu chuẩn.
  - **5 Thẻ Chỉ Số KPI Trực Quan**: Mô-men lớn nhất ($\tau_{max}$), Tải motor cao nhất (%), Tải trọng đầu gắp ($m_{payload}$), **Tổng công suất tiêu thụ ($P_{total}$)**, và Trạng thái an toàn hệ thống.
  - **Mini Cartesian HUD Tọa Độ Đầu Gắp (TCP)**: Hiển thị ngay trên đầu Tab 1 tọa độ vị trí $X, Y, Z$ (mm), tầm vươn $R$, bán kính làm việc $r_{xy}$, và góc xoay Roll, Pitch, Yaw.
  - **Bảng 10 Cột Chi Tiết Từng Khớp**:
    - Tên khớp ($J_1 \dots J_5$) và vai trò trục.
    - Vị trí góc khớp $q$ (rad & độ $^\circ$).
    - Vận tốc góc $\dot{q}$ (rad/s & độ/s).
    - Gia tốc góc $\ddot{q}$ (rad/s²).
    - Mô-men xoắn tức thời $\tau$ (N·m) tính toán trực tiếp từ thuật toán RNEA.
    - Giới hạn mô-men định mức $\tau_{max}$ của từng động cơ.
    - Thanh đo % tải trọng động cơ kèm mã màu trực quan: 🟢 Xanh ($<50\%$), 🟡 Vàng ($50-80\%$), 🔴 Đỏ ($>80\%$).
    - **Công suất cơ/điện tức thời $P_i = |\tau_i \cdot \dot{q}_i|$ (Watts)**.
    - Huy hiệu trạng thái: `AN TOÀN`, `CẢNH BÁO`, `QUÁ TẢI`.
  - **Thanh Thao Tác Nhanh**:
    - Nhập tải gắp (Payload) $0 - 7.0\text{ kg}$ (thay đổi sẽ lập tức cập nhật lại mô-men xoắn bù tải).
    - Các nút lệnh: `Chạy Vòng Lặp`, `Chạy 1 Chiều`, `Dừng Robot`, `Về Home 0°`.
    - **Nút "📸 Xuất Báo Cáo CSV (Snapshot)"**: Xuất toàn bộ dữ liệu 5 khớp và tọa độ TCP ra file CSV kèm lưu trực tiếp vào Clipboard.
* **Tab 2: Phân Tích Động Lực Học Nghịch (Inverse Dynamics - ID)**:
  - Thuật toán đệ quy **RNEA (Recursive Newton-Euler Algorithm)** tính toán mô-men xoắn yêu cầu $\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) + \tau_f + J^T F_{ext}$.
  - Phân tích chi tiết từng thành phần trên bảng **Nền Trắng**: Lực quán tính $M\ddot{q}$, Lực Coriolis & ly tâm $C\dot{q}$, Trọng lực $g(q)$, Lực do tải trọng $J^TF$, Ma sát $\tau_f$.
  - **Bộ Nút Tư Thế Mẫu (Pose Presets)**: `Home 0°`, `Vươn Ngang 90°`, `Gập Khuỷu 90°`, `Sẵn Sàng` (Ready Pose) để kiểm thử nhanh mô-men với 1 cú click.
  - Nút *"⚖️ Cân bằng trọng lực tĩnh"* để tính nhanh mô-men giữ cánh tay chống rơi tự do.
* **Tab 3: Mô Phỏng Động Lực Học Thuận (Forward Dynamics - FD)**:
  - Giải bài toán gia tốc góc khớp: $\ddot{q} = M(q)^{-1} (\tau - C(q, \dot{q})\dot{q} - g(q) - \tau_f - J^T F_{ext})$.
  - Thanh trượt chỉnh mô-men xoắn $\tau_1 \dots \tau_5$ (N·m) cho từng khớp.
  - Bảng kết quả gia tốc $\ddot{q}$ và ma trận khối lượng đối xứng xác định dương $M(q)$ $5 \times 5$.
  - **Hiển thị Động năng toàn robot $E_k = \frac{1}{2}\dot{q}^T M(q)\dot{q}$ (Joules)**.
  - **Mô phỏng tương tác vật lý thời gian thực trên RViz2 (20Hz)**: Kéo thanh trượt mô-men xoắn, cánh tay robot trong RViz2 chuyển động theo đúng quy luật động lực học Newton!
* **Tab 4: Động Học D-H & Tọa Độ Điểm Cuối (Kinematics & D-H Matrix)**:
  - **4 Thẻ Chỉ Số Không Gian**: Tọa độ TCP $(X, Y, Z)$, Tầm vươn $R$ ($r_{xy}$), Góc Euler Roll-Pitch-Yaw ($^\circ$), Trạng thái vùng làm việc Workspace Envelope.
  - **Bảng Tham Số D-H Chuẩn (Standard DH Table)**: Hiển thị và cập nhật thời gian thực các tham số $\theta_i$ (độ & rad), $d_i$ (m), $a_i$ (m), $\alpha_i$ ($^\circ$), vector trục quay $Z_i$.
  - **Ma Trận Biến Đổi Đồng Nhất $4 \times 4$ (Homogeneous Transform)**: Bảng ma trận $4 \times 4$ trực quan, cho phép chuyển đổi xem ma trận tổng thể $T_0^5$ (Base $\to$ TCP) hoặc từng ma trận mắt xích $A_1 \dots A_5$.
* **5. Trực Quan Hóa Động Lực Học Trên RViz2**:
  - Tự động hiển thị qua topic `/denso/joint_dynamics_markers`:
    + **Bảng Đo Lường Động Lực Học Tập Trung (Unified Telemetry Table)**: Đặt gọn gàng cố định bên cạnh robot, hiển thị bảng chữ nhật chuẩn monospace ASCII đầy đủ 5 khớp, Torque, Limit, Load %, Status, Accel, tự động xoay theo camera và đổi màu cảnh báo (🟢 Xanh, 🟡 Vàng, 🔴 Đỏ).
    + **Mũi tên vector mô-men xoắn 3D (Torque Arrows)**: Căn chỉnh thanh mảnh trên 5 trục quay vật lý của robot, biểu diễn vector xoay của lực mà không gây vướng mắt.

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
