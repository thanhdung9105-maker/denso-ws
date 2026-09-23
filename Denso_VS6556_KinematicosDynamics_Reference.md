# Tài Liệu Khối Lượng, Quán Tính & Động Lực Học - Denso VS-6556 Robot

Tài liệu này ghi nhận toàn bộ thông số khối lượng, khối tâm (Center of Mass - CoM), tensor quán tính (Inertia Tensor) và các tham số động lực học mô phỏng được cấu hình cho robot công nghiệp **Denso VS-6556** (5 bậc tự do / 5-DOF).

---

## 1. Bảng Khối Lượng & Quán Tính Các Part (Links)

Hệ thống tọa độ cơ sở tuân theo chuẩn **URDF ROS 2 / Gazebo Sim**:
- Đơn vị khối lượng: Kilogram ($\text{kg}$)
- Đơn vị tọa độ khối tâm: Mét ($\text{m}$)
- Đơn vị mô-men quán tính: $\text{kg}\cdot\text{m}^2$

| Part / Link Name | 3D CAD Mesh | Khối lượng $m$ ($\text{kg}$) | Khối tâm CoM $(x, y, z)$ [m] | Ma trận quán tính chính $(I_{xx}, I_{yy}, I_{zz})$ [$\text{kg}\cdot\text{m}^2$] | Vai trò kết cấu |
|---|---|:---:|:---:|:---:|---|
| **`base_link`** | `410590-0080-c001.STL` | **8.0** | $(0.000, 0.000, 0.090)$ | $(0.040, 0.040, 0.050)$ | Đế cố định robot, chứa mạch cấp nguồn và bearing trục 1 |
| **`link_1`** | `410590-0080-c002.STL` | **4.5** | $(-0.040, 0.000, 0.080)$ | $(0.035, 0.035, 0.025)$ | Khớp vai xoay trục Z (Shoulder Yaw - J1) |
| **`link_2`** | `410590-0080-c003.STL` | **4.2** | $(0.000, 0.003, 0.135)$ | $(0.045, 0.040, 0.015)$ | Cánh tay trên (Upper Arm - J2), chịu lực uốn trọng trường lớn nhất |
| **`link_3`** | `410590-0080-c004.STL` | **3.1** | $(-0.040, 0.020, 0.040)$ | $(0.020, 0.020, 0.010)$ | Cụm khuỷu tay (Elbow Pitch - J3) |
| **`link_4`** | `410590-0080-c005.STL` | **2.0** | $(-0.100, 0.000, 0.000)$ | $(0.005, 0.015, 0.015)$ | Cánh tay trước xoay tròn (Forearm Roll - J4) |
| **`link_5`** | `410590-0080-c006.STL` | **1.2** | $(-0.030, 0.000, 0.000)$ | $(0.003, 0.003, 0.003)$ | Mặt bích gắn khâu công tác cuối (Tool Flange / Wrist Pitch - J5) |

**Tổng khối lượng mô hình hiện tại:** **$23.0\text{ kg}$**

---

## 2. So Sánh Với Robot Denso VS-6556 Thực Tế (Datasheet)

Theo tài liệu tiêu chuẩn từ nhà sản xuất **Denso Robotics (VS-6556 Series)**:

- **Khối lượng toàn thân robot thực tế (Arm weight):** $\approx 35\text{ kg}$
- **Tải trọng nâng danh định (Rated payload):** $6.0\text{ kg}$ (Tối đa $7.0\text{ kg}$)
- **Tầm với cánh tay (Reach radius):** $653\text{ mm}$ (Bán kính vùng làm việc)
- **Độ lặp lại vị trí (Repeatability):** $\pm 0.02\text{ mm}$

### 🔍 Giải thích sự khác biệt $23\text{ kg}$ vs $35\text{ kg}$:
1. **Lõi động cơ & Hộp số vi sai:** Robot công nghiệp thực tế tích hợp các động cơ AC Servo tải cao kèm hộp số giảm tốc sóng Harmonic Drive bằng thép đặc nặng khoảng $8 - 10\text{ kg}$ bố trí tập trung sâu trong thân đế và vai.
2. **Cụm phanh từ (Electromagnetic Brakes) & Bó cáp ngầm:** Mỗi khớp robot thực tế đều có cuộn hút cơ khí chống rơi khi ngắt điện, nặng khoảng $2\text{ kg}$.
3. **Mô hình hóa số:** Trong mô phỏng ROS 2 / Gazebo, việc đặt khối lượng mô hình ở mức $23\text{ kg}$ (tương ứng kết cấu vỏ nhôm đúc chịu lực chính) giúp ma trận quán tính điều kiện tốt (well-conditioned inertia matrix), ngăn chặn hiện tượng mất ổn định số (numerical explosion) khi chạy mô phỏng thời gian thực qua CPU ảo hóa WSL2.

---

## 3. Bảng Thông Số Động Cơ AC Servo & Hộp Số Giảm Tốc (Datasheet No. 410590-0080)

Toàn bộ 5 khớp của Denso VS-6556 đều được dẫn động bằng động cơ **AC Servomotor đồng bộ nam châm vĩnh cửu** kết hợp với **Hộp số giảm tốc sóng Harmonic Drive / RV Reducer** và cảm biến góc quay **Encoder tuyệt đối (Absolute Encoder)**:

| Khớp (Joint) | Công suất định mức | Tốc độ động cơ | Vận tốc góc cực đại (Datasheet) | Vận tốc góc cực đại (rad/s) | Tỷ số truyền danh định | Mô-men định mức / Max Effort | Max Acceleration | Phanh giữ cơ khí (Brake) |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **`joint_1`** (Shoulder Yaw) | **400 W** | 3000 rpm | $262.5^\circ/	ext{s}$ | **$4.5815	ext{ rad/s}$** | $pprox 80:1$ | **$80	ext{ Nm}$** | $15.0	ext{ rad/s}^2$ | Phanh tùy chọn (Bản BW) |
| **`joint_2`** (Shoulder Pitch) | **750 W** | 3000 rpm | $240.0^\circ/	ext{s}$ | **$4.1888	ext{ rad/s}$** | $pprox 100:1$ | **$100	ext{ Nm}$** | $12.0	ext{ rad/s}^2$ | **Có** (Tiêu chuẩn chống rơi) |
| **`joint_3`** (Elbow Pitch) | **400 W** | 3000 rpm | $300.0^\circ/	ext{s}$ | **$5.2360	ext{ rad/s}$** | $pprox 80:1$ | **$60	ext{ Nm}$** | $15.0	ext{ rad/s}^2$ | **Có** (Tiêu chuẩn chống rơi) |
| **`joint_4`** (Forearm Roll) | **100 W** | 3000 rpm | $300.0^\circ/	ext{s}$ | **$5.2360	ext{ rad/s}$** | $pprox 50:1$ | **$25	ext{ Nm}$** (Allowable $16.2	ext{ Nm}$) | $20.0	ext{ rad/s}^2$ | **Có** (Tiêu chuẩn) |
| **`joint_5`** (Wrist Pitch) | **100 W** | 3000 rpm | $300.0^\circ/	ext{s}$ | **$5.2360	ext{ rad/s}$** | $pprox 50:1$ | **$20	ext{ Nm}$** (Allowable $16.2	ext{ Nm}$) | $20.0	ext{ rad/s}^2$ | Có (Bản B) |

> [!NOTE]
> **Đặc điểm phân bổ công suất:**
> - **Khớp 2 (Vai / Shoulder):** Đòi hỏi công suất lớn nhất ($750	ext{ W}$) và mô-men lớn nhất ($100	ext{ Nm}$) do phải chịu toàn bộ tải trọng uốn của các khâu 2, 3, 4, 5 cùng vật nâng $7	ext{ kg}$ ở cánh tay đòn cực đại $653	ext{ mm}$.
> - **Khớp 4 & 5 (Cổ tay / Wrist):** Cụm cổ tay có thiết kế siêu mỏng nhẹ ($110	ext{ mm}$ bề rộng), sử dụng động cơ $100	ext{ W}$ công suất cao với mô-men cho phép giới hạn ở mức $16.2	ext{ Nm}$ (quán tính cho phép $0.413	ext{ kg}\cdot	ext{m}^2$).

---

## 4. Bảng Tham Số Động Lực Học Khớp (Joint Dynamics)

Các giá trị ma sát, giảm chấn và giới hạn mô-men xoắn được đồng bộ chính xác giữa Gazebo Harmonic và thuật toán Recursive Newton-Euler trong `denso_dynamics_engine.py`:

| Khớp (Joint) | Giới hạn góc quay [rad] | Max Effort ($\text{Nm}$) | Viscous Damping $B$ ($\text{Nm}\cdot\text{s/rad}$) | Coulomb Friction $F_c$ ($\text{Nm}$) |
|---|:---:|:---:|:---:|:---:|
| **`joint_1`** (Shoulder Yaw) | $[-2.967, +2.967]$ ($\pm 170^\circ$) | **80** | **0.8** | **0.5** |
| **`joint_2`** (Shoulder Pitch) | $[-2.094, +2.356]$ ($-120^\circ \rightarrow +135^\circ$) | **100** | **1.2** | **0.8** |
| **`joint_3`** (Elbow Pitch) | $[-2.077, +2.897]$ ($-119^\circ \rightarrow +166^\circ$) | **60** | **0.9** | **0.6** |
| **`joint_4`** (Forearm Roll) | $[-3.316, +3.316]$ ($\pm 190^\circ$) | **25** | **0.3** | **0.2** |
| **`joint_5`** (Wrist Pitch) | $[-2.094, +2.094]$ ($\pm 120^\circ$) | **20** | **0.2** | **0.15** |

---

## 5. Cải Tiến Cấu Hình Gazebo Harmonic

1. **Đồng bộ tần số điều khiển (500 Hz):**
   - Physics timestep trong `denso_world.sdf`: $0.002\text{ s}$ ($500\text{ Hz}$).
   - `controller_manager` update_rate trong `denso_controllers.yaml`: Được nâng từ $100\text{ Hz} \rightarrow 500\text{ Hz}$.
   - Triệt tiêu cảnh báo lệch chu kỳ (*"Desired controller update period is slower than simulation period"*).
2. **Độ lợi tỷ lệ vị trí `gz_ros2_control`:**
   - Cấu hình `position_proportional_gain: 0.5` trong URDF plugin và yaml controller.
   - Thời hằng đáp ứng: $T = \frac{1}{0.5 \times 500} = 0.004\text{ s} = 4\text{ ms}$ (phản hồi tức thời, loại bỏ độ rơ/trễ khi mang tải).
3. **ODE Quick Solver & Ràng buộc vật lý:**
   - Solver iterations: `50`, SOR: `1.3`.
   - Contact CFM: `0.00001`, ERP: `0.2`.
   - Đảm bảo các khớp liên kết cứng vững, không rung giật vi mô khi dừng giữ vị trí cố định.
