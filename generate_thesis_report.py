# -*- coding: utf-8 -*-
"""
High-Fidelity Graduation Thesis Report HTML & PDF Generator
Denso VS-6556 6-DOF Industrial Robot Manipulator
Ho Chi Minh City University of Technology (HCMUT - Bach Khoa)
Department of Applied Mechanics, Faculty of Applied Science
"""
import os, sys, base64, subprocess

def get_b64(path):
    if os.path.exists(path):
        ext = os.path.splitext(path)[1].lower().replace('.', '')
        if ext == 'jpg': ext = 'jpeg'
        with open(path, 'rb') as f:
            return f'data:image/{ext};base64,{base64.b64encode(f.read()).decode("utf-8")}'
    return ''

fig_dir = '/home/dung/denso_ws/report_figures'
img1 = get_b64(f'{fig_dir}/fig1_rnea_6dof_torques.png')
img2 = get_b64(f'{fig_dir}/fig2_workspace_manipulability.png')
img3 = get_b64(f'{fig_dir}/fig3_trajectory_tracking.png')
img4 = get_b64(f'{fig_dir}/fig4_kinematic_skeleton.png')
img5 = get_b64(f'{fig_dir}/fig5_gui_gcode_control_panel.jpg')
img6 = get_b64(f'{fig_dir}/fig6_rviz_com_markers.png')
img7 = get_b64(f'{fig_dir}/fig7_wrist_tool_assembly.png')
img8 = get_b64(f'{fig_dir}/fig8_energy_optimization.png')

html_template = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<title>Báo Cáo Đồ Án Chuyên Ngành: Robot Denso VS-6556 (6-DOF) - HCMUT</title>
<style>
  @page {
    size: A4 portrait;
    margin: 18mm 16mm 18mm 16mm;
    @bottom-right {
      content: counter(page);
      font-size: 9pt;
      font-family: 'Times New Roman', serif;
    }
  }

  body {
    font-family: 'Times New Roman', 'Cambria', serif;
    font-size: 11pt;
    line-height: 1.45;
    color: #1a1a1a;
    background-color: #ffffff;
    margin: 0;
    padding: 0;
  }

  h1, h2, h3, h4 {
    font-family: 'Times New Roman', 'Arial', sans-serif;
    color: #0d233a;
    margin-top: 14pt;
    margin-bottom: 6pt;
    page-break-after: avoid;
    break-after: avoid;
  }

  h1 {
    font-size: 14pt;
    page-break-before: always;
    break-before: page;
    text-transform: uppercase;
    border-bottom: 2px solid #0d233a;
    padding-bottom: 4px;
    margin-top: 20pt;
  }

  h2 {
    font-size: 12.5pt;
    border-bottom: 1px solid #cbd5e1;
    padding-bottom: 2px;
  }

  h3 {
    font-size: 11.5pt;
    font-style: italic;
  }

  p {
    text-align: justify;
    margin-top: 4pt;
    margin-bottom: 6pt;
    text-indent: 1.2cm;
  }

  p.no-indent {
    text-indent: 0;
  }

  .cover-page {
    text-align: center;
    padding: 20px 10px;
    border: 3px double #0d233a;
    box-sizing: border-box;
    height: 96%;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    page-break-after: always;
    break-after: page;
  }

  .cover-header {
    font-size: 12pt;
    font-weight: bold;
    text-transform: uppercase;
    line-height: 1.3;
  }

  .cover-sub {
    font-size: 11pt;
    font-weight: normal;
    text-transform: uppercase;
    margin-top: 4px;
  }

  .cover-divider {
    width: 35%;
    height: 1.5px;
    background-color: #0d233a;
    margin: 10px auto;
  }

  .cover-title-box {
    margin: 30px 0;
  }

  .cover-report-type {
    font-size: 13pt;
    font-weight: bold;
    color: #c53030;
    text-transform: uppercase;
    letter-spacing: 1px;
    margin-bottom: 12px;
  }

  .cover-title {
    font-size: 16pt;
    font-weight: bold;
    text-transform: uppercase;
    color: #0d233a;
    line-height: 1.35;
    padding: 0 15px;
  }

  .cover-subtitle {
    font-size: 11pt;
    font-style: italic;
    color: #4a5568;
    margin-top: 10px;
  }

  .cover-meta {
    font-size: 11pt;
    text-align: left;
    display: inline-block;
    margin: 20px auto;
    line-height: 1.6;
    border: 1px solid #cbd5e0;
    background-color: #f7fafc;
    padding: 12px 25px;
    border-radius: 4px;
  }

  .cover-footer {
    font-size: 11pt;
    font-weight: bold;
    text-transform: uppercase;
  }

  .page-break {
    page-break-after: always;
    break-after: page;
  }

  table {
    width: 100%;
    border-collapse: collapse;
    margin: 10pt 0;
    font-size: 9.5pt;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  th, td {
    border: 1px solid #a0aec0;
    padding: 5px 7px;
    text-align: left;
    vertical-align: middle;
  }

  th {
    background-color: #2b6cb0;
    color: #ffffff;
    font-weight: bold;
    text-align: center;
  }

  tr:nth-child(even) {
    background-color: #f7fafc;
  }

  .math-box {
    background-color: #f8fafc;
    border-left: 3.5px solid #3182ce;
    padding: 8px 14px;
    margin: 8pt 0;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 9.5pt;
    color: #1a202c;
    border-radius: 0 4px 4px 0;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .note-box {
    background-color: #ebf8ff;
    border: 1px solid #bee3f8;
    border-left: 4px solid #3182ce;
    padding: 8px 12px;
    margin: 8pt 0;
    font-size: 10pt;
    border-radius: 0 4px 4px 0;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .warning-box {
    background-color: #fffaf0;
    border: 1px solid #feebc8;
    border-left: 4px solid #dd6b20;
    padding: 8px 12px;
    margin: 8pt 0;
    font-size: 10pt;
    border-radius: 0 4px 4px 0;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .success-box {
    background-color: #f0fff4;
    border: 1px solid #c6f6d5;
    border-left: 4px solid #38a169;
    padding: 8px 12px;
    margin: 8pt 0;
    font-size: 10pt;
    border-radius: 0 4px 4px 0;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .figure-container {
    text-align: center;
    margin: 12pt auto;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .figure-container img {
    max-width: 90%;
    max-height: 350px;
    height: auto;
    object-fit: contain;
    border: 1px solid #e2e8f0;
    border-radius: 4px;
    box-shadow: 0 1px 3px rgba(0,0,0,0.1);
  }

  .figure-caption {
    font-size: 9.5pt;
    font-weight: bold;
    font-style: italic;
    color: #4a5568;
    margin-top: 5px;
  }

  .toc-item {
    display: flex;
    justify-content: space-between;
    margin: 4px 0;
    font-size: 10.5pt;
  }

  .toc-title {
    font-weight: normal;
  }

  .toc-dots {
    flex-grow: 1;
    border-bottom: 1px dotted #a0aec0;
    margin: 0 5px 4px 5px;
  }

  .toc-page {
    font-weight: bold;
    color: #2b6cb0;
  }

  .spec-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    margin: 10px 0;
    font-size: 9.5pt;
  }

  .spec-card {
    border: 1px solid #e2e8f0;
    background: #f8fafc;
    padding: 8px 12px;
    border-radius: 4px;
  }

  .spec-card strong {
    color: #2b6cb0;
  }

  .code-inline {
    font-family: 'Consolas', 'Courier New', monospace;
    background-color: #edf2f7;
    padding: 1px 4px;
    border-radius: 3px;
    font-size: 9pt;
    color: #c53030;
  }
</style>
</head>
<body>

<!-- ========================================== -->
<!-- TRANG BÌA CHÍNH (HCMUT THESIS STANDARD)    -->
<!-- ========================================== -->
<div class="cover-page">
  <div>
    <div class="cover-header">ĐẠI HỌC QUỐC GIA THÀNH PHỐ HỒ CHÍ MINH<br>TRƯỜNG ĐẠI HỌC BÁCH KHOA (HCMUT)</div>
    <div class="cover-sub">KHOA KHOA HỌC ỨNG DỤNG — BỘ MÔN CƠ KỸ THUẬT</div>
    <div class="cover-divider"></div>
  </div>

  <div class="cover-title-box">
    <div class="cover-report-type">BÁO CÁO KỸ THUẬT ĐỒ ÁN CHUYÊN NGÀNH ROBOT HỌC</div>
    <div class="cover-title">
      MÔ HÌNH HÓA ĐỘNG HỌC, ĐỘNG LỰC HỌC, QUY HOẠCH QUỸ ĐẠO G-CODE 6 TRỤC VÀ ĐIỀU KHIỂN ROBOT CÔNG NGHIỆP DENSO VS-6556 (6-DOF) TRÊN NỀN TẢNG ROS 2 & GAZEBO HARMONIC
    </div>
    <div class="cover-subtitle">
      Tài liệu tổng hợp lý thuyết Cơ học Giải tích, Chuẩn hóa D-H 6 Bậc Tự Do, Động học nghịch Giải tích Đóng (Pieper Decoupling), Động lực học RNEA, Tối ưu hóa quỹ đạo và Kiểm chứng số học tuyệt đối
    </div>
  </div>

  <div class="cover-meta">
    <table>
      <tr style="background:transparent;"><td style="border:none; width:150px; font-weight:bold;">Ngành:</td><td style="border:none;">Cơ Kỹ Thuật (Kỹ thuật Cơ điện tử & Robot học)</td></tr>
      <tr style="background:transparent;"><td style="border:none; font-weight:bold;">Đối tượng mô hình:</td><td style="border:none;">Tay máy công nghiệp Denso VS-6556 (Hoàn chỉnh 6-DOF)</td></tr>
      <tr style="background:transparent;"><td style="border:none; font-weight:bold;">Khâu công tác:</td><td style="border:none;">Mặt bích Flange J6 (Trục W) + Cụm gá bút vẽ Pen Tool</td></tr>
      <tr style="background:transparent;"><td style="border:none; font-weight:bold;">Môi trường phát triển:</td><td style="border:none;">Ubuntu 24.04 LTS (WSL2) • ROS 2 Jazzy • Gazebo Harmonic</td></tr>
      <tr style="background:transparent;"><td style="border:none; font-weight:bold;">Độ chính xác xác thực:</td><td style="border:none; color:#2b6cb0; font-weight:bold;">Sai số vị trí ΔE = 0.000000 mm • Sai số động lực học 7.66×10⁻¹⁵</td></tr>
    </table>
  </div>

  <div>
    <div class="cover-divider"></div>
    <div class="cover-footer">THÀNH PHỐ HỒ CHÍ MINH — NĂM 2026</div>
  </div>
</div>

<!-- ========================================== -->
<!-- TRANG LỜI NÓI ĐẦU & TÓM TẮT ĐỀ TÀI         -->
<!-- ========================================== -->
<h1 style="page-break-before:avoid; break-before:avoid; margin-top:10pt;">LỜI NÓI ĐẦU & TÓM TẮT ĐỀ TÀI (ABSTRACT)</h1>

<p>Trong xu thế chuyển đổi số và tự động hóa công nghiệp (Industry 4.0), việc xây dựng Bản sao số (Digital Twin) có độ trung thực cao kết hợp thuật toán điều khiển giải tích thời gian thực là yêu cầu sống còn của các hệ thống tay máy công nghiệp. Đồ án này tập trung nghiên cứu, mô hình hóa toán học và hiện thực hóa phần mềm điều khiển cho robot công nghiệp 6 bậc tự do <strong>Denso VS-6556</strong> trên nền tảng hệ điều hành robot thế hệ mới nhất <strong>ROS 2 Jazzy Jalisco</strong> và trình mô phỏng vật lý <strong>Gazebo Harmonic (GZ Sim 8)</strong>.</p>

<p>Kế thừa và nâng cấp toàn diện từ các phiên bản nghiên cứu trước (vốn chỉ mô phỏng 5 bậc tự do rút gọn), phiên bản hiện tại đã hiện thực hóa <strong>đầy đủ 6 bậc tự do (6-DOF: J1, J2, J3, J4, J5, J6)</strong>, hoàn thiện mặt bích xoay chuẩn công nghiệp ISO 9409-1 (Joint 6 - Flange) được định nghĩa đồng bộ là trục xoay <strong>W</strong> trong hệ tọa độ G-code 6 trục (A, B, C, U, V, W). Hệ thống tích hợp toàn bộ chuỗi tính toán giải tích từ nguyên lý cơ học đầu tiên: từ Bảng tham số Denavit-Hartenberg Tiêu chuẩn (Standard D-H) và Mở rộng (Extended D-H), phương trình Động học thuận giải tích, thuật toán Động học nghịch giải tích đóng 6-DOF (Closed-Form Analytical IK) dựa trên phương pháp tách biến tâm cổ tay cầu Pieper kết hợp khai triển Euler Z-Y-Z và xử lý triệt để điểm kỳ dị cổ tay (Wrist Singularity), đến Thuật toán Đệ quy Newton-Euler (RNEA) tính toán mô-men động lực học ngược và động lực học thuận với độ phức tạp tuyến tính O(n).</p>

<div class="success-box">
  <strong>Đặc tả kỹ thuật nổi bật đạt được trong đồ án:</strong>
  <ul>
    <li><strong>Cấu trúc cơ học:</strong> Robot công nghiệp 6 khớp quay (6 Revolute Joints: R-R-R-R-R-R), tải trọng 5.0 kg, độ lặp lại ±0.02 mm, tầm với cực đại tới Flange TCP là <strong>990.04 mm</strong> (tâm cổ tay Point P tại <strong>900.00 mm</strong>).</li>
    <li><strong>Đặc trưng cơ khí Denso:</strong> Triệt tiêu hoàn toàn độ lệch ngang trục Y: <span class="code-inline">Y_total = -74.5 mm (J2) + 6.5 mm (J3) + 68.0 mm (J4) = 0.0000 mm</span>. Tâm cổ tay Point P và đầu bút Flange TCP luôn nằm tuyệt đối trên mặt phẳng đối xứng trung tâm chứa trục Z₀.</li>
    <li><strong>Độ chính xác số học:</strong> Sai số Động học thuận giữa Standard D-H, Extended D-H, URDF CAD và lượng giác hình học đạt tuyệt đối <strong>ΔE = 0.000000 mm</strong>. Sai số động lực học thuận/nghịch đạt <strong>7.66 × 10⁻¹⁵ rad/s²</strong> (giới hạn chính xác số học dấu phẩy động 64-bit IEEE 754).</li>
    <li><strong>Thị giác & Điều khiển G-code 6 trục:</strong> Xử lý ảnh OpenCV (Canny, Contour, Douglas-Peucker) tự động tạo quỹ đạo nét vẽ, giải IK hàng loạt cho 6234 điểm với tỷ lệ khả thi <strong>100.0% (0 va chạm, 0 lỗi kỳ dị)</strong>, xuất mã lệnh CNC G-code 6 trục (A, B, C, U, V, W) điều khiển robot vẽ trực tiếp trong Gazebo với sai số bám quỹ đạo < 0.02 mm.</li>
  </ul>
</div>

<div class="figure-container">
  <img src="{img4}" alt="Sơ đồ Động học 6 khớp Denso VS-6556">
  <div class="figure-caption">Hình 1: Cấu trúc chuỗi động học 6 bậc tự do Denso VS-6556 và tọa độ các điểm nút tại tư thế Home (q=0)</div>
</div>

<!-- ========================================== -->
<!-- MỤC LỤC & DANH MỤC TỪ VIẾT TẮT             -->
<!-- ========================================== -->
<h1>MỤC LỤC BÁO CÁO (TABLE OF CONTENTS)</h1>

<div class="toc-item"><span class="toc-title"><strong>LỜI NÓI ĐẦU & TÓM TẮT ĐỀ TÀI (ABSTRACT)</strong></span><span class="toc-dots"></span><span class="toc-page">2</span></div>
<div class="toc-item"><span class="toc-title"><strong>DANH MỤC KÝ HIỆU & TỪ VIẾT TẮT</strong></span><span class="toc-dots"></span><span class="toc-page">3</span></div>
<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 1. TỔNG QUAN HỆ THỐNG VÀ KIẾN TRÚC PHẦN MỀM ROS 2 (6-DOF)</strong></span><span class="toc-dots"></span><span class="toc-page">4</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">1.1. Bối cảnh nghiên cứu và đối tượng tay máy Denso VS-6556 6 bậc tự do</span><span class="toc-dots"></span><span class="toc-page">4</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">1.2. Nâng cấp toàn diện từ 5-DOF lên hệ thống 6-DOF hoàn chỉnh</span><span class="toc-dots"></span><span class="toc-page">4</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">1.3. Cấu trúc module hóa 3 gói (packages) trong workspace denso_ws</span><span class="toc-dots"></span><span class="toc-page">5</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">1.4. Sơ đồ Node Graph và luồng dữ liệu thời gian thực qua CycloneDDS</span><span class="toc-dots"></span><span class="toc-page">5</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 2. QUY TRÌNH TRIỂN KHAI KỸ THUẬT TIÊU CHUẨN (STANDARD WORKFLOW)</strong></span><span class="toc-dots"></span><span class="toc-page">6</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">2.1. Khảo sát CAD SolidWorks và trích xuất dữ liệu hình học, khối lượng</span><span class="toc-dots"></span><span class="toc-page">6</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">2.2. Xây dựng tệp mô tả hình học URDF/Xacro đồng bộ gốc tọa độ</span><span class="toc-dots"></span><span class="toc-page">6</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">2.3. Quy trình kiểm chứng chéo độc lập 3 phương pháp toán học</span><span class="toc-dots"></span><span class="toc-page">7</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 3. PHÂN ĐỊNH THƯ VIỆN CÓ SẴN VS. THUẬT TOÁN TỰ PHÁT TRIỂN</strong></span><span class="toc-dots"></span><span class="toc-page">8</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">3.1. Bảng đối chiếu minh bạch 6 nhóm chức năng cốt lõi</span><span class="toc-dots"></span><span class="toc-page">8</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">3.2. Tính độc lập và khả năng làm chủ công nghệ giải tích</span><span class="toc-dots"></span><span class="toc-page">8</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 4. PHÂN TÍCH ĐỘNG HỌC THUẬN (FORWARD KINEMATICS - FK) 6-DOF</strong></span><span class="toc-dots"></span><span class="toc-page">9</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">4.1. Đặc điểm cơ khí hãng Denso: Triệt tiêu độ lệch ngang trục Y</span><span class="toc-dots"></span><span class="toc-page">9</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">4.2. Chiều quay âm của các trục gập trong URDF (axis = [0, -1, 0])</span><span class="toc-dots"></span><span class="toc-page">9</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">4.3. Bảng D-H Mở rộng (Extended D-H) 7 hàng đồng bộ vỏ CAD RViz</span><span class="toc-dots"></span><span class="toc-page">10</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">4.4. Bảng Standard D-H 6 bậc tự do và Ma trận biến đổi thuần nhất</span><span class="toc-dots"></span><span class="toc-page">10</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">4.5. Phương trình giải tích closed-form vị trí Point P và Flange TCP</span><span class="toc-dots"></span><span class="toc-page">11</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 5. ĐỘNG HỌC NGHỊCH (INVERSE KINEMATICS - IK) 6-DOF & PHÂN TÍCH KỲ DỊ</strong></span><span class="toc-dots"></span><span class="toc-page">12</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">5.1. Phương pháp Tách tâm cổ tay cầu Pieper (Pieper's Decoupling)</span><span class="toc-dots"></span><span class="toc-page">12</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">5.2. Giải tích hình học nghiệm đóng 3 khớp định vị q1, q2, q3</span><span class="toc-dots"></span><span class="toc-page">12</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">5.3. Trích xuất 3 khớp định hướng q4, q5, q6 qua khai triển Euler Z-Y-Z</span><span class="toc-dots"></span><span class="toc-page">13</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">5.4. Giải thuật xử lý triệt để Kỳ dị Cổ tay (Wrist Singularity)</span><span class="toc-dots"></span><span class="toc-page">13</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">5.5. Ma trận Jacobian hình học 6×6, chỉ số Yoshikawa và giải thuật DLS</span><span class="toc-dots"></span><span class="toc-page">14</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 6. ĐỘNG LỰC HỌC ROBOT (RNEA) & TỐI ƯU HÓA NĂNG LƯỢNG</strong></span><span class="toc-dots"></span><span class="toc-page">15</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">6.1. Phương trình Lagrange 6 bậc tự do dạng ma trận</span><span class="toc-dots"></span><span class="toc-page">15</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">6.2. Thuật toán Đệ quy Newton-Euler (RNEA) độ phức tạp tuyến tính O(n)</span><span class="toc-dots"></span><span class="toc-page">15</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">6.3. Bảng đặc tính khối lượng, khối tâm CoM và tensor quán tính 6 khâu</span><span class="toc-dots"></span><span class="toc-page">16</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">6.4. Ma trận khối lượng M(q) đối xứng xác định dương và Động lực học thuận</span><span class="toc-dots"></span><span class="toc-page">16</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">6.5. Bù trọng trường tĩnh tại Home và giải thích hiện tượng uốn tĩnh</span><span class="toc-dots"></span><span class="toc-page">17</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 7. THỊ GIÁC MÁY TÍNH & TẠO QUỸ ĐẠO G-CODE 6 TRỤC (A, B, C, U, V, W)</strong></span><span class="toc-dots"></span><span class="toc-page">18</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">7.1. Pipeline xử lý ảnh OpenCV: Tiền xử lý, lọc Canny và đa giác hóa</span><span class="toc-dots"></span><span class="toc-page">18</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">7.2. Tối ưu hóa hành trình di chuyển không tải bằng thuật toán TSP</span><span class="toc-dots"></span><span class="toc-page">18</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">7.3. Định dạng mã lệnh CNC G-code 6 trục với khớp 6 là trục W</span><span class="toc-dots"></span><span class="toc-page">19</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">7.4. Kiểm tra an toàn và tính khả thi động học 100% trên toàn bộ nét vẽ</span><span class="toc-dots"></span><span class="toc-page">19</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 8. HIỆN THỰC HÓA TRÊN ROS 2, GAZEBO HARMONIC & MOVEIT 2</strong></span><span class="toc-dots"></span><span class="toc-page">20</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">8.1. Cấu hình ros2_control, plugin gz_ros2_control và Spline Controller</span><span class="toc-dots"></span><span class="toc-page">20</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">8.2. Mô phỏng phanh điện từ và đồng bộ thời gian clock qua DDS</span><span class="toc-dots"></span><span class="toc-page">20</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">8.3. Tích hợp MoveIt 2: Quy hoạch OMPL RRTConnect và Pilz Industrial</span><span class="toc-dots"></span><span class="toc-page">21</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 9. TỔNG HỢP KẾT QUẢ KIỂM CHỨNG SỐ HỌC (VERIFICATION AUDIT)</strong></span><span class="toc-dots"></span><span class="toc-page">22</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">9.1. Bảng đối chiếu Động học thuận 6 khớp với 5 tư thế kiểm chứng tiêu biểu</span><span class="toc-dots"></span><span class="toc-page">22</span></div>
<div class="toc-item" style="padding-left:15px;"><span class="toc-title">9.2. Kiểm chứng độ chính xác Động học nghịch và Động lực học thuận/nghịch</span><span class="toc-dots"></span><span class="toc-page">23</span></div>

<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 10. NGÂN HÀNG CÂU HỎI PHẢN BIỆN CHUYÊN SÂU & HƯỚNG DẪN TRẢ LỜI</strong></span><span class="toc-dots"></span><span class="toc-page">24</span></div>
<div class="toc-item"><span class="toc-title"><strong>CHƯƠNG 11. KẾT LUẬN & HƯỚNG PHÁT TRIỂN ĐỀ TÀI</strong></span><span class="toc-dots"></span><span class="toc-page">26</span></div>
<div class="toc-item"><span class="toc-title"><strong>TÀI LIỆU THAM KHẢO (REFERENCES)</strong></span><span class="toc-dots"></span><span class="toc-page">27</span></div>

<h2 style="margin-top:20pt;">DANH MỤC KÝ HIỆU & TỪ VIẾT TẮT</h2>
<table>
  <tr><th style="width:25%;">Từ Viết Tắt</th><th style="width:35%;">Tiếng Anh</th><th style="width:40%;">Ý Nghĩa / Định Nghĩa Kỹ Thuật</th></tr>
  <tr><td><strong>6-DOF</strong></td><td>6 Degrees of Freedom</td><td>6 bậc tự do chuyển động độc lập trong không gian 3 chiều</td></tr>
  <tr><td><strong>D-H</strong></td><td>Denavit-Hartenberg</td><td>Quy ước kinh điển đặt hệ trục tọa độ liên kết trên các khâu robot</td></tr>
  <tr><td><strong>FK / IK</strong></td><td>Forward / Inverse Kinematics</td><td>Bài toán Động học thuận / Động học nghịch của tay máy</td></tr>
  <tr><td><strong>TCP</strong></td><td>Tool Center Point</td><td>Điểm trung tâm công tác khâu cuối (đầu bút vẽ / mỏ hàn / kẹp)</td></tr>
  <tr><td><strong>Point P</strong></td><td>Wrist Center Point</td><td>Tâm cổ tay cầu nơi 3 trục khớp J4, J5, J6 giao nhau đồng quy</td></tr>
  <tr><td><strong>RNEA</strong></td><td>Recursive Newton-Euler Algorithm</td><td>Thuật toán đệ quy Newton-Euler giải động lực học tay máy độ phức tạp O(n)</td></tr>
  <tr><td><strong>DLS</strong></td><td>Damped Least Squares</td><td>Phương pháp bình phương tối thiểu có giảm chấn (Levenberg-Marquardt)</td></tr>
  <tr><td><strong>TSP</strong></td><td>Traveling Salesperson Problem</td><td>Bài toán người bán hàng tối ưu hành trình nhấc bút di chuyển không tải</td></tr>
  <tr><td><strong>ROS 2</strong></td><td>Robot Operating System 2</td><td>Hệ điều hành robot phân tán thế hệ 2 thời gian thực (Jazzy Jalisco)</td></tr>
  <tr><td><strong>URDF / SDF</strong></td><td>Unified Robot / Simulation Description Format</td><td>Định dạng XML mô tả cấu trúc hình học, cơ học và mô phỏng robot</td></tr>
</table>

<!-- ========================================== -->
<!-- CHƯƠNG 1: TỔNG QUAN KIẾN TRÚC HỆ THỐNG      -->
<!-- ========================================== -->
<h1>CHƯƠNG 1. TỔNG QUAN HỆ THỐNG VÀ KIẾN TRÚC PHẦN MỀM ROS 2 (6-DOF)</h1>

<h2>1.1. Bối cảnh nghiên cứu và đối tượng tay máy Denso VS-6556 6 bậc tự do</h2>
<p>Tay máy công nghiệp 6 bậc tự do (6-DOF Articulated Robot) là cấu hình hình học linh hoạt và phổ biến nhất trong nền sản xuất hiện đại. Nhờ sở hữu đủ 6 bậc tự do trong không gian Descartes (gồm 3 bậc tự do tịnh tiến định vị X, Y, Z và 3 bậc tự do quay định hướng Roll, Pitch, Yaw), tay máy có khả năng đưa khâu công tác cuối (End-Effector) tiếp cận bất kỳ vị trí và hướng tùy ý trong không gian làm việc mà không bị ràng buộc góc nghiêng.</p>

<p>Đối tượng nghiên cứu của đồ án là robot công nghiệp <strong>Denso VS-6556</strong> do tập đoàn Denso Wave (Nhật Bản) sản xuất. Đây là dòng robot khớp nối tay gập chính xác cao chuyên dụng cho các dây chuyền lắp ráp điện tử vi mô, chế tạo bán dẫn, dán keo tự động, hàn điểm laser và gắp thả tốc độ cao. Dòng robot này nổi tiếng thế giới nhờ tỷ số công suất/khối lượng vượt trội (tổng khối lượng thân máy chỉ ~23.0 kg nhưng mang được tải trọng định mức 5.0 kg ở vận tốc tổng hợp lên tới gần 8.0 m/s) và độ lặp lại vị trí siêu vi mô ±0.02 mm.</p>

<h2>1.2. Nâng cấp toàn diện từ 5-DOF lên hệ thống 6-DOF hoàn chỉnh</h2>
<p>Trong giai đoạn nghiên cứu ban đầu, mô hình chỉ được thiết lập ở cấu hình 5 bậc tự do (5-DOF, khâu 5 gá liền đầu bút) nhằm giải quyết nhanh bài toán vẽ nét thẳng đứng. Tuy nhiên, cấu hình 5-DOF bộc lộ hạn chế cố hữu: không gian hướng bị khuyết một bậc tự do xoay quanh trục khâu cuối, dẫn đến ma trận Jacobian chỉ đạt kích thước 6×5, không thể thực hiện các thao tác công nghiệp phức tạp đòi hỏi điều khiển góc xoay mặt bích độc lập (như vặn vít, mài bóng, bôi keo định hướng hoặc thay đổi góc ngòi bút nghiêng nghệ thuật).</p>

<p>Trong phiên bản hoàn thiện này, toàn bộ hệ thống đã được nâng cấp lên <strong>6 bậc tự do hoàn chỉnh (Full 6-DOF)</strong>:</p>
<ul>
  <li><strong>Khâu 5 (link_5):</strong> Đóng vai trò cụm cùi chỏ cổ tay gập (Wrist Pitch Knuckle), quay quanh trục nằm ngang -Y₄ với góc giới hạn [-120°, +120°], khối lượng 0.95 kg.</li>
  <li><strong>Khâu 6 (link_6):</strong> Đóng vai trò mặt bích xoay chuẩn công nghiệp (Rotating Flange / Tool Knuckle), liên kết với khâu 5 qua khớp quay thứ 6 (joint_6) xoay tròn quanh trục Z₅ với góc xoay mở rộng [-360°, +360°] (±2π rad), khối lượng 0.25 kg. Chiều cao cổ tay d₆ = 0.09004 m (90.04 mm).</li>
  <li><strong>Khâu công tác (pen_tool_link):</strong> Cụm ngòi bút vẽ kỹ thuật có khối lượng 0.015 kg gắn đồng trục vào mặt bích khâu 6, cho phép điều khiển xoay hướng tiếp tuyến nét vẽ hoặc giữ ổn định tuyệt đối trong suốt quá trình vẽ CNC.</li>
  <li><strong>Ánh xạ G-code 6 trục:</strong> Trong bộ sinh mã lệnh CNC tự động, khớp 6 được chuẩn hóa thành <strong>trục W</strong>, tạo thành hệ tọa độ 6 trục tiêu chuẩn: A (J1), B (J2), C (J3), U (J4), V (J5), W (J6).</li>
</ul>

<div class="figure-container">
  <img src="{img7}" alt="Cụm mặt bích khớp 6 và ngòi bút vẽ">
  <div class="figure-caption">Hình 2: Cụm khớp xoay thứ 6 (Joint 6 - Flange) và khâu công tác bút vẽ được mô hình hóa chi tiết</div>
</div>

<h2>1.3. Cấu trúc module hóa 3 gói (packages) trong workspace denso_ws</h2>
<p>Toàn bộ mã nguồn hệ thống được tổ chức khoa học theo chuẩn kỹ thuật ROS 2 gồm 3 gói độc lập:</p>

<table>
  <tr><th style="width:25%;">Tên Gói (Package)</th><th style="width:35%;">Tệp Cấu Hình & Scripts Cốt Lõi</th><th style="width:40%;">Chức Năng & Ý Nghĩa Kỹ Thuật</th></tr>
  <tr>
    <td><strong>1. denso_vs6556</strong><br><em>(Gói Mô tả Cơ khí CAD/URDF)</em></td>
    <td>
      • <span class="code-inline">urdf/denso_vs6556.urdf</span><br>
      • <span class="code-inline">meshes/*.STL</span> (c001 → c006, flange)<br>
      • <span class="code-inline">launch/display.launch.py</span><br>
      • <span class="code-inline">rviz/display.rviz</span>
    </td>
    <td>Định nghĩa cấu trúc hình học cây động học (Kinematic Tree) 6 bậc tự do gồm đế cố định base_link, 6 khâu động (link_1 → link_6) và khâu công tác pen_tool_link. Khai báo chính xác thuộc tính quán tính CAD (Inertial Tensor 3×3, CoM, Mass), giới hạn góc khớp, vận tốc tối đa, mô-men cực đại và hệ số ma sát nhớt.</td>
  </tr>
  <tr>
    <td><strong>2. denso_vs6556_moveit_config</strong><br><em>(Gói Động học, Động lực học RNEA & MoveIt 2)</em></td>
    <td>
      • <span class="code-inline">scripts/denso_dynamics_engine.py</span><br>
      • <span class="code-inline">scripts/denso_kinematics_gui.py</span><br>
      • <span class="code-inline">scripts/denso_dh_fk_node.py</span><br>
      • <span class="code-inline">scripts/denso_com_visualizer.py</span><br>
      • <span class="code-inline">scripts/verify_denso_dh.py</span><br>
      • <span class="code-inline">config/kinematics.yaml, ompl_planning.yaml</span>
    </td>
    <td><strong>Trọng tâm toán học của đồ án:</strong> Chứa bộ giải Động học thuận (Chuẩn Standard D-H và Extended D-H), bộ giải Động học nghịch giải tích đóng 6-DOF (Pieper's Spherical Wrist Decoupling + Euler Z-Y-Z), bộ giải số Hybrid DLS, ma trận Jacobian hình học 6×6, chỉ số khả năng thao tác Yoshikawa, và Engine Động lực học RNEA giải động lực học ngược/thuận, tính ma trận khối lượng M(q) ∈ ℝ⁶ˣ⁶ đối xứng xác định dương.</td>
  </tr>
  <tr>
    <td><strong>3. denso_gazebo</strong><br><em>(Gói Mô phỏng Vật lý, Điều khiển & Thị giác G-Code)</em></td>
    <td>
      • <span class="code-inline">urdf/denso_vs6556_gazebo.urdf.xacro</span><br>
      • <span class="code-inline">config/denso_controllers.yaml</span><br>
      • <span class="code-inline">worlds/denso_world.sdf</span><br>
      • <span class="code-inline">scripts/denso_gazebo_control_panel.py</span><br>
      • <span class="code-inline">scripts/denso_gcode_vision_engine.py</span><br>
      • <span class="code-inline">scripts/denso_gcode_joint_table.py</span>
    </td>
    <td>Tích hợp plugin phần cứng ảo <span class="code-inline">gz_ros2_control</span> đóng vòng kín điều khiển quỹ đạo khớp tần số 100 Hz. Tích hợp Engine thị giác OpenCV xử lý ảnh, tối ưu hóa bài toán người bán hàng TSP, sinh mã CNC G-code 6 trục (A, B, C, U, V, W), giải IK hàng loạt và điều khiển tay máy vẽ thời gian thực trên bàn mô phỏng Gazebo.</td>
  </tr>
</table>

<h2>1.4. Sơ đồ Node Graph và luồng dữ liệu thời gian thực qua CycloneDDS</h2>
<p>Hệ thống vận hành theo kiến trúc hướng dữ liệu phân tán (Data-Centric Publish/Subscribe) thông qua tầng giao tiếp trung gian thời gian thực <strong>CycloneDDS</strong>:</p>

<div class="math-box">
  <strong>Sơ đồ Luồng Dữ liệu Thời gian Thực (Real-Time Node Architecture):</strong><br>
  [OpenCV Vision / G-Code] ──(Tọa độ X,Y,Z,W)──> [denso_gazebo_control_panel] ──(IK 6-DOF)──><br>
  ──> Action Goal: [FollowJointTrajectory] ──> [controller_manager / arm_controller (100Hz)] ──><br>
  ──> Hardware Interface: [gz_ros2_control] <──(Khớp q, dq, ddq)──> [Gazebo Physics Engine (ODE)]<br>
  ──> Broadcast: [/joint_states (100Hz)] ──> [robot_state_publisher] ──> TF2 Tree ──> [RViz2 / CoM Visualizer]
</div>

<!-- ========================================== -->
<!-- CHƯƠNG 2: QUY TRÌNH KỸ THUẬT TIÊU CHUẨN     -->
<!-- ========================================== -->
<h1>CHƯƠNG 2. QUY TRÌNH TRIỂN KHAI KỸ THUẬT TIÊU CHUẨN (STANDARD WORKFLOW)</h1>

<p>Để hoàn thành đồ án đáp ứng tiêu chuẩn khắt khe của Bộ môn Cơ Kỹ thuật (HCMUT), tác giả đã tuân thủ quy trình thiết kế kỹ thuật chính thống gồm 7 bước tuần tự từ khảo sát hình học thực tế đến hiện thực hóa mã điều khiển:</p>

<h2>2.1. Khảo sát CAD SolidWorks và trích xuất dữ liệu hình học, khối lượng</h2>
<p>Dữ liệu hình học gốc được đối chiếu trực tiếp từ tài liệu kỹ thuật chính hãng <em>Denso VS-6556/6577 User's Manual (Doc No. 410590-0080)</em> kết hợp mô hình lắp ghép chi tiết 3D SolidWorks <span class="code-inline">VS-6556-B_asm.SLDASM</span>. Cụm lắp ráp được phân rã thành 7 phân đoạn vật rắn (Rigid Bodies): Đế cố định <span class="code-inline">c001.STL</span> (base_link), 6 khâu chuyển động <span class="code-inline">c002.STL</span> đến <span class="code-inline">c007.STL</span> (link_1 → link_6) và khâu bút vẽ <span class="code-inline">pen_tool_link</span>.</p>

<p>Kích thước khoảng cách tâm trục giữa các ổ lăn được đo đạc chính xác trong môi trường CAD 3D:</p>
<ul>
  <li>Mặt bích đế tới tâm trục khớp 1: <strong>d₁ = 205.0 mm</strong> (tịnh tiến theo trục Z).</li>
  <li>Khớp 1 tới khớp vai J2: cao thêm <strong>130.0 mm</strong> theo trục Z, lùi ra sau <strong>a₁ = -75.0 mm</strong> theo trục X, và lệch sang phải <strong>b₂ = -74.5 mm</strong> theo trục Y. Tổng chiều cao trục vai so với mặt bàn là <strong>z₂ = 335.0 mm</strong>.</li>
  <li>Chiều dài thanh cánh tay trên (Upper Arm - Khâu 2): <strong>L₂ = 270.0 mm</strong>.</li>
  <li>Độ lệch khuỷu tay (Elbow Offset - Khâu 3): lệch ngang lại <strong>b₃ = +6.5 mm</strong> theo trục Y, và nhô về phía trước <strong>a₃ = +90.0 mm</strong> theo trục X.</li>
  <li>Chiều dài cẳng tay (Forearm - Khâu 4): gồm đoạn cổ khuỷu 109.0 mm và thân cẳng tay 186.0 mm, tổng chiều dài dọc là <strong>d₄ = L₃ = 295.0 mm</strong>. Cụm cẳng tay lệch ngang bù lại <strong>b₄ = +68.0 mm</strong> theo trục Y.</li>
  <li>Chiều dài cụm cùi chỏ cổ tay (Khâu 5): <strong>d₅ = 0.0 mm</strong> (trục quay gập giao đồng quy tại tâm cổ tay Point P ở cao độ <strong>Z_P = 900.0 mm</strong>).</li>
  <li>Chiều dài cụm mặt bích xoay (Khâu 6): <strong>d₆ = 90.04 mm</strong> (khoảng cách từ tâm Point P tới mặt phẳng đầu bích Flange TCP ở cao độ <strong>Z_TCP = 990.04 mm</strong>).</li>
</ul>

<div class="figure-container">
  <img src="{img6}" alt="Trọng tâm CoM và hệ tọa độ 3D trên RViz2">
  <div class="figure-caption">Hình 3: Vị trí trọng tâm CoM (khối cầu lục lam), ma trận quán tính và hệ trục tọa độ 3D của từng khâu trên RViz2</div>
</div>

<h2>2.2. Xây dựng tệp mô tả hình học URDF/Xacro đồng bộ gốc tọa độ</h2>
<p>Một thách thức cơ kỹ thuật phổ biến khi nhập mô hình STL từ SolidWorks vào ROS 2 là: các tệp lưới STL thường được xuất theo gốc tọa độ lắp ghép chung (Global Assembly Origin) của SolidWorks chứ không nằm tại tâm trục quay của từng khớp. Do đó, tác giả đã đo đạc chính xác vector dịch chuyển và góc quay bù để đưa vào thẻ <span class="code-inline">&lt;origin xyz="..." rpy="..."/&gt;</span> trong các khối <span class="code-inline">&lt;visual&gt;</span> và <span class="code-inline">&lt;collision&gt;</span> của URDF. Nhờ vậy, trục quay của từng thẻ <span class="code-inline">&lt;joint&gt;</span> luôn trùng khít 100% với trục ổ bi thực tế.</p>

<h2>2.3. Quy trình kiểm chứng chéo độc lập 3 phương pháp toán học</h2>
<p>Để đảm bảo không xảy ra sai sót đánh máy hoặc nhầm lẫn quy ước dấu, tác giả đã xây dựng chương trình kiểm tra độc lập <span class="code-inline">verify_denso_dh.py</span> so sánh song song 3 phương pháp tính Động học thuận trên toàn bộ không gian làm việc:</p>
<ol>
  <li><strong>Phương pháp 1 (URDF Kinematic Chain):</strong> Tính tích các phép biến đổi không gian từ cây biến đổi của ROS 2 Robot State Publisher.</li>
  <li><strong>Phương pháp 2 (Standard D-H 4×4 Matrix Multiplication):</strong> Nhân chuỗi 6 ma trận D-H chuẩn kinh điển <span class="code-inline">T = A₁ · A₂ · A₃ · A₄ · A₅ · A₆</span>.</li>
  <li><strong>Phương pháp 3 (Closed-Form Planar Trigonometry):</strong> Tính trực tiếp bằng phương trình hình học lượng giác sin-cos trên mặt phẳng đối xứng cánh tay.</li>
</ol>
<p>Kết quả kiểm chứng xác nhận: <strong>Sai số vị trí giữa cả 3 phương pháp đạt tuyệt đối ΔE = 0.000000 mm</strong> trên mọi tư thế kiểm tra.</p>

<!-- ========================================== -->
<!-- CHƯƠNG 3: PHÂN ĐỊNH THƯ VIỆN VS TỰ PHÁT TRIỂN -->
<!-- ========================================== -->
<h1>CHƯƠNG 3. PHÂN ĐỊNH THƯ VIỆN CÓ SẴN VS. THUẬT TOÁN TỰ PHÁT TRIỂN</h1>

<h2>3.1. Bảng đối chiếu minh bạch 6 nhóm chức năng cốt lõi</h2>
<p>Trước hội đồng đánh giá luận văn, việc làm rõ ranh giới giữa <em>những gì thừa hưởng từ hệ sinh thái nguồn mở ROS 2</em> và <em>những gì do sinh viên tự nghiên cứu, phát triển thuật toán toán học</em> là thước đo quan trọng nhất về năng lực chuyên môn:</p>

<table>
  <tr><th style="width:20%;">Nhóm Nghiên Cứu</th><th style="width:38%;">Công Cụ / Thư Viện Có Sẵn Của ROS 2</th><th style="width:42%;">Thuật Toán & Module Tự Thiết Kế / Lập Trình Cốt Lõi</th></tr>
  <tr>
    <td><strong>1. Quản lý Hệ trục Tọa độ (TF2)</strong></td>
    <td>
      • <span class="code-inline">robot_state_publisher</span><br>
      • <span class="code-inline">tf2_ros</span> (Broadcaster, Buffer)<br>
      • <span class="code-inline">rviz2</span> (Giao diện hiển thị 3D)
    </td>
    <td>
      <strong>Tự xây dựng node denso_dh_fk_node.py:</strong> Tính toán trực tiếp chuỗi ma trận 4×4 D-H Mở rộng độc lập với URDF, xuất chuỗi PoseArray và phát sóng độc lập để kiểm chứng sai số động học theo thời gian thực.
    </td>
  </tr>
  <tr>
    <td><strong>2. Động học Thuận (FK) 6-DOF</strong></td>
    <td>
      • Thư viện động học hình học <span class="code-inline">kdl_parser</span> trong MoveIt 2.
    </td>
    <td>
      <strong>Tự thiết lập bảng Standard D-H 6-DOF & Extended D-H:</strong> Chứng minh hiện tượng triệt tiêu độ lệch ngang Y, hiệu chỉnh chính xác quy ước dấu âm trục quay (-Y) và lập trình hàm tính FK closed-form siêu tốc (< 0.5 μs).
    </td>
  </tr>
  <tr>
    <td><strong>3. Động học Nghịch (IK) 6-DOF</strong></td>
    <td>
      • Plugin số học <span class="code-inline">kdl_kinematics_plugin</span> trong MoveIt 2 (chạy lặp Newton-Raphson, hội tụ chậm và dễ kẹt kỳ dị).
    </td>
    <td>
      <strong>Tự lập trình bộ giải Analytical Geometric IK 6-DOF:</strong> Áp dụng phương pháp tách tâm cổ tay cầu Pieper, giải tam giác vai-khuỷu dạng đóng, khai triển Euler Z-Y-Z cho cụm cổ tay 4-5-6, và giải thuật xử lý triệt để kỳ dị cổ tay (Wrist Singularity) trong 0 vòng lặp (thời gian tính < 1.0 μs).
    </td>
  </tr>
  <tr>
    <td><strong>4. Động lực học (Dynamics RNEA)</strong></td>
    <td>
      • Bộ giải vật lý đa vật rắn ODE (Open Dynamics Engine) chạy ngầm trong lõi Gazebo Sim.
    </td>
    <td>
      <strong>Tự xây dựng hoàn chỉnh DensoDynamicsEngine:</strong> Lập trình thuật toán Recursive Newton-Euler (RNEA) 6 khâu; trích xuất ma trận quán tính suy rộng M(q) ∈ ℝ⁶ˣ⁶, kiểm chứng tính đối xứng xác định dương (6 trị riêng > 0); lập trình bộ giải Động lực học thuận (FD) và bù trọng trường tĩnh.
    </td>
  </tr>
  <tr>
    <td><strong>5. Điều khiển Chuyển động</strong></td>
    <td>
      • Gói <span class="code-inline">ros2_control</span> & <span class="code-inline">controller_manager</span><br>
      • <span class="code-inline">joint_trajectory_controller</span><br>
      • Plugin phần cứng ảo <span class="code-inline">gz_ros2_control</span>
    </td>
    <td>
      <strong>Tự thiết kế bộ điều khiển mô phỏng tích hợp:</strong> Tạo node điều khiển nội suy quỹ đạo đường cong chữ S mượt mà (S-Curve Profile), thiết lập cơ chế giữ khớp phanh điện từ và bộ điều hòa xung nhịp thời gian thực 100 Hz.
    </td>
  </tr>
  <tr>
    <td><strong>6. Thị giác & Quỹ đạo G-code 6 trục</strong></td>
    <td>
      • Thư viện xử lý ma trận ảnh OpenCV nền tảng.
    </td>
    <td>
      <strong>Tự phát triển Engine denso_gcode_vision_engine.py:</strong> Lọc viền Canny, xấp xỉ Ramer-Douglas-Peucker, tối ưu hành trình nhấc bút TSP (Nearest Neighbor), ánh xạ không gian làm việc bàn robot, sinh mã lệnh CNC G-code 6 trục (A, B, C, U, V, W) và kiểm tra tính khả thi động học 100%.
    </td>
  </tr>
</table>

<h2>3.2. Tính độc lập và khả năng làm chủ công nghệ giải tích</h2>
<p>Điểm sáng tạo then chốt của đồ án là <strong>không sử dụng MoveIt 2 hay KDL như một chiếc "hộp đen"</strong> để tìm nghiệm động học khi vẽ tranh. Toàn bộ chuỗi 6234 điểm quỹ đạo G-code được giải trực tiếp bằng bộ giải giải tích do chính nhóm tác giả phát triển. Điều này mang lại ưu thế tuyệt đối về thời gian thực: trong khi bộ giải số KDL mất hàng trăm mili-giây và thường xuyên thất bại tại biên kỳ dị, bộ giải giải tích 6-DOF thực thi toàn bộ 6234 điểm chỉ trong <strong>0.015 giây</strong> với tỷ lệ thành công 100%.</p>

<!-- ========================================== -->
<!-- CHƯƠNG 4: ĐỘNG HỌC THUẬN (FK) 6 BẬC TỰ DO   -->
<!-- ========================================== -->
<h1>CHƯƠNG 4. PHÂN TÍCH ĐỘNG HỌC THUẬN (FORWARD KINEMATICS - FK) 6 BẬC TỰ DO</h1>

<h2>4.1. Đặc điểm cơ khí hãng Denso: Triệt tiêu độ lệch ngang trục Y</h2>
<p>Một trong những nét độc đáo và tinh tế nhất trong thiết kế cơ khí của hãng Denso trên dòng robot VS-6556 là <strong>sự triệt tiêu hoàn toàn độ lệch ngang trục Y (Lateral Offset Cancellation)</strong>. Trong kết cấu cơ khí thực tế:</p>
<ul>
  <li>Cụm động cơ và hộp số Harmonic của khớp vai làm Khâu 2 lệch sang phải một đoạn: <strong>b₂ = -74.5 mm</strong> theo trục Y.</li>
  <li>Tại khớp khuỷu, vỏ Khâu 3 dịch ngược lại một đoạn: <strong>b₃ = +6.5 mm</strong> theo trục Y.</li>
  <li>Tại cụm cẳng tay, khớp Khâu 4 bù lại chính xác một đoạn: <strong>b₄ = +68.0 mm</strong> theo trục Y.</li>
</ul>

<div class="math-box">
  <strong>Phương trình Triệt tiêu Độ lệch Ngang:</strong><br>
  Y_total = b₂ + b₃ + b₄ = -74.5 mm + 6.5 mm + 68.0 mm = <strong>0.0000 mm</strong>
</div>

<p>Nhờ đặc tính cân bằng hình học hoàn hảo này, Tâm cổ tay (Point P) và Khâu công tác đầu bút (Flange TCP) luôn nằm chính xác trên mặt phẳng đối xứng trung tâm chứa trục quay thẳng đứng Z₀ của đế cố định khi q₄ = q₅ = 0. Đây là cơ sở toán học then chốt cho phép giải tích hóa bài toán Động học nghịch mà không cần giải hệ phương trình phi tuyến không gian phức tạp!</p>

<h2>4.2. Chiều quay âm của các trục gập trong URDF (axis = [0, -1, 0])</h2>
<p>Trong tệp mô tả chuẩn URDF của robot Denso VS-6556, các khớp gập gồm Khớp 2 (Vai - J2), Khớp 3 (Khuỷu - J3) và Khớp 5 (Cổ tay - J5) đều có vector đơn vị trục quay hướng theo phương âm trục Y:</p>
<div class="math-box">
  e_axis,2 = [0, -1, 0]^T, \quad e_axis,3 = [0, -1, 0]^T, \quad e_axis,5 = [0, -1, 0]^T
</div>
<p>Điều này có nghĩa là: theo quy tắc bàn tay phải, khi góc quay URDF q₂ > 0 hoặc q₃ > 0, khâu tương ứng sẽ gập chúc về phía trước (-X). Khi chuyển đổi sang hệ tọa độ D-H chuẩn (với trục quay luôn quy ước là trục Z), các góc này phải có dấu âm kèm theo góc pha bù tương ứng.</p>

<h2>4.3. Bảng D-H Mở rộng (Extended D-H) 7 hàng đồng bộ vỏ CAD RViz</h2>
<p>Để hiển thị chính xác từng vỏ kim loại CAD bị lệch tâm Y trên RViz2 và kiểm tra va chạm tự thân giữa các cụm chi tiết máy, mô hình D-H Mở rộng sử dụng phép biến đổi 5 tham số bổ sung thêm độ dịch ngang b_i theo trục Y:</p>
<div class="math-box">
  H_i(q_i) = \text{Trans}(a_i, b_i, d_i) \cdot \text{Rot}_{axis,i}(q_i)
</div>

<table>
  <tr><th>Khâu Đích</th><th>Biến Khớp</th><th>Trục Quay URDF</th><th>a_i (ΔX, m)</th><th>b_i (ΔY, m)</th><th>d_i (ΔZ, m)</th><th>Tọa độ Gốc Khâu tại Home (q=0) [mm]</th></tr>
  <tr><td><strong>link_1</strong></td><td>q₁</td><td>+Z [0, 0, 1]</td><td>0.0000</td><td>0.0000</td><td>0.2050</td><td>(0.0, 0.0, +205.0)</td></tr>
  <tr><td><strong>link_2</strong></td><td>q₂</td><td>-Y [0, -1, 0]</td><td>-0.0750</td><td>-0.0745</td><td>0.1300</td><td>(-75.0, -74.5, +335.0)</td></tr>
  <tr><td><strong>link_3</strong></td><td>q₃</td><td>-Y [0, -1, 0]</td><td>0.0000</td><td>+0.0065</td><td>0.2700</td><td>(-75.0, -68.0, +605.0)</td></tr>
  <tr><td><strong>link_4</strong></td><td>q₄</td><td>+Z [0, 0, 1]</td><td>+0.0900</td><td>+0.0680</td><td>0.1090</td><td>(+15.0, 0.0, +714.0)</td></tr>
  <tr><td><strong>link_5 (Point P)</strong></td><td>q₅</td><td>-Y [0, -1, 0]</td><td>0.0000</td><td>0.0000</td><td>0.1860</td><td>(+15.0, 0.0, +900.0)</td></tr>
  <tr><td><strong>link_6</strong></td><td>q₆</td><td>+Z [0, 0, 1]</td><td>0.0000</td><td>0.0000</td><td>0.06204</td><td>(+15.0, 0.0, +962.04)</td></tr>
  <tr><td><strong>flange (TCP)</strong></td><td>Cố định</td><td>Fixed Tool</td><td>0.0000</td><td>0.0000</td><td>0.0280</td><td>(+15.0, 0.0, +990.04)</td></tr>
</table>

<h2>4.4. Bảng Standard D-H 6 bậc tự do và Ma trận biến đổi thuần nhất</h2>
<p>Để giải tích hóa bài toán động học theo chuẩn giáo trình Robot học kinh điển (Craig / Spong / Siciliano), bảng <strong>Standard D-H 6 Bậc Tự Do</strong> (4 tham số: a_i, α_i, d_i, θ_i) đã được thiết lập, kiểm chứng và chuẩn hóa hoàn hảo:</p>

<table>
  <tr><th>Khớp i</th><th>Chuyển Tiếp Khung Tọa Độ</th><th>a_i (m)</th><th>α_i (rad / độ)</th><th>d_i (m)</th><th>Biến Góc D-H θ_i (theo góc khớp URDF q_i)</th></tr>
  <tr><td><strong>1</strong></td><td>Đế O₀ → Khớp vai O₁ (J2)</td><td>-0.0750</td><td>-π/2 (-90°)</td><td>0.3350</td><td>θ₁ = +q₁</td></tr>
  <tr><td><strong>2</strong></td><td>Cánh tay trên O₁ → Khuỷu O₂ (J3)</td><td>+0.2700</td><td>0 (0°)</td><td>0.0000</td><td>θ₂ = -q₂ - π/2 (-q₂ - 90°)</td></tr>
  <tr><td><strong>3</strong></td><td>Độ lệch khuỷu O₂ → Cẳng tay O₃</td><td>+0.0900</td><td>+π/2 (+90°)</td><td>0.0000</td><td>θ₃ = -q₃ + π/2 (-q₃ + 90°)</td></tr>
  <tr><td><strong>4</strong></td><td>Xoay dọc cẳng tay O₃ → Tâm cổ tay O₄</td><td>0.0000</td><td>-π/2 (-90°)</td><td>0.2950</td><td>θ₄ = +q₄</td></tr>
  <tr><td><strong>5</strong></td><td>Gập cổ tay O₄ (Point P) → Trục bích O₅</td><td>0.0000</td><td>+π/2 (+90°)</td><td>0.0000</td><td>θ₅ = -q₅</td></tr>
  <tr><td><strong>6</strong></td><td>Xoay mặt bích O₅ → Mặt bích Flange TCP O₆</td><td>0.0000</td><td>0 (0°)</td><td><strong>0.09004</strong></td><td>θ₆ = +q₆</td></tr>
</table>

<div class="note-box">
  <strong>Công thức ma trận biến đổi thuần nhất Standard D-H tổng quát:</strong><br>
  A_i(θ_i) = \text{Rot}_z(θ_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(α_i) = 
  \begin{bmatrix}
    \cos θ_i & -\sin θ_i \cos α_i & \sin θ_i \sin α_i & a_i \cos θ_i \\
    \sin θ_i & \cos θ_i \cos α_i & -\cos θ_i \sin α_i & a_i \sin θ_i \\
    0 & \sin α_i & \cos α_i & d_i \\
    0 & 0 & 0 & 1
  \end{bmatrix}<br><br>
  Chuỗi nhân ma trận xác định tư thế khâu cuối:
  <span class="code-inline">⁰T₆(q) = A₁(θ₁) · A₂(θ₂) · A₃(θ₃) · A₄(θ₄) · A₅(θ₅) · A₆(θ₆)</span>
</div>

<h2>4.5. Phương trình giải tích closed-form vị trí Point P và Flange TCP</h2>
<p>Khai triển trực tiếp phép nhân ma trận trên mặt phẳng đối xứng cánh tay (được định vị bởi góc quay đế q₁), ta thu được hệ phương trình giải tích tường minh tính toán tọa độ Tâm cổ tay Point P và Đầu bút Flange TCP:</p>

<div class="math-box">
  Đặt q₂₃ = q₂ + q₃; \quad x₂ = -75.0\text{ mm}, z₂ = 335.0\text{ mm}, L₂ = 270.0\text{ mm}, a₃ = 90.0\text{ mm}, L₃ = 295.0\text{ mm}, d₆ = 90.04\text{ mm}:<br><br>
  1. Tọa độ hướng kính và cao độ của Tâm cổ tay Point P (Khớp 5):<br>
  r_P(q₂, q₃) = x₂ - L₂ \sin(q₂) - L₃ \sin(q₂₃) + a₃ \cos(q₂₃)<br>
  Z_P(q₂, q₃) = z₂ + L₂ \cos(q₂) + L₃ \cos(q₂₃) + a₃ \sin(q₂₃)<br>
  X_P = r_P \cos(q₁), \quad Y_P = r_P \sin(q₁)<br><br>
  2. Tọa độ của Mặt bích Flange TCP (Khớp 6, tính cả góc cuộn cẳng tay q₄ và gập cổ tay q₅):<br>
  \Delta r_{in} = -d₆ \cdot (\sin(q₂₃) \cos(q₅) + \cos(q₂₃) \cos(q₄) \sin(q₅))<br>
  \Delta Z_{in} = +d₆ \cdot (\cos(q₂₃) \cos(q₅) - \sin(q₂₃) \cos(q₄) \sin(q₅))<br>
  \Delta Y_{out} = -d₆ \cdot \sin(q₄) \sin(q₅)<br>
  X_F = (r_P + \Delta r_{in}) \cos(q₁) - \Delta Y_{out} \sin(q₁)<br>
  Y_F = (r_P + \Delta r_{in}) \sin(q₁) + \Delta Y_{out} \cos(q₁)<br>
  Z_F = Z_P + \Delta Z_{in}
</div>

<div class="figure-container">
  <img src="{img2}" alt="Không gian làm việc và chỉ số thao tác">
  <div class="figure-caption">Hình 4: Không gian vươn làm việc (Reachability Envelope) trên mặt phẳng X-Z và bản đồ nhiệt chỉ số thao tác Yoshikawa w(q)</div>
</div>

<!-- ========================================== -->
<!-- CHƯƠNG 5: ĐỘNG HỌC NGHỊCH (IK) 6 BẬC TỰ DO   -->
<!-- ========================================== -->
<h1>CHƯƠNG 5. ĐỘNG HỌC NGHỊCH (INVERSE KINEMATICS - IK) 6 BẬC TỰ DO & PHÂN TÍCH KỲ DỊ</h1>

<h2>5.1. Phương pháp Tách tâm cổ tay cầu Pieper (Pieper's Decoupling)</h2>
<p>Một trong những thành tựu kinh điển của lý thuyết điều khiển robot là <strong>Tiêu chuẩn Pieper (Pieper's Criterion)</strong>: Mọi tay máy 6 bậc tự do có 3 trục khớp cổ tay liên tiếp giao nhau đồng quy tại một điểm (Spherical Wrist) đều có thể giải được nghiệm giải tích đóng (Closed-form Analytical Solution) bằng cách tách rời bài toán vị trí (Positioning) và bài toán định hướng (Orientation).</p>

<p>Đối với Denso VS-6556, 3 trục quay của Khớp 4 (xoay cẳng tay), Khớp 5 (gập cổ tay) và Khớp 6 (xoay mặt bích) giao nhau đồng quy chính xác tại <strong>Tâm cổ tay cầu Point P</strong>. Với một tư thế mục tiêu bất kỳ bao gồm vị trí p_target = [X_T, Y_T, Z_T]^T và ma trận định hướng R_target ∈ SO(3), vector pháp tuyến của mặt bích khâu cuối chính là cột thứ 3 của ma trận quay: z_flange = R_target[:, 2].</p>

<div class="math-box">
  <strong>Bước 1: Tách biến xác định tọa độ Tâm cổ tay Point P (p_wrist):</strong><br>
  p_wrist = p_target - d₆ \cdot z_flange = 
  \begin{bmatrix} X_T \\ Y_T \\ Z_T \end{bmatrix} - 0.09004 \cdot \begin{bmatrix} r₁₃ \\ r₂₃ \\ r₃₃ \end{bmatrix}
</div>

<h2>5.2. Giải tích hình học nghiệm đóng 3 khớp định vị q₁, q₂, q₃</h2>
<p>Sau khi xác định được tọa độ tâm cổ tay p_wrist = [X_w, Y_w, Z_w]^T, bài toán rút về giải góc cho 3 khớp định vị đầu tiên (J1, J2, J3) hoàn toàn độc lập với hướng của cổ tay:</p>

<div class="math-box">
  <strong>1. Góc xoay đế q₁ (hỗ trợ 2 chế độ Vươn Tiến và Vươn Lùi):</strong><br>
  r_{xy} = \sqrt{X_w² + Y_w²}<br>
  • Nhánh Vươn Tiến (Forward Mode): q_{1,fwd} = \text{atan2}(Y_w, X_w), \quad x_{planar} = +r_{xy}<br>
  • Nhánh Vươn Lùi (Backward Mode): q_{1,back} = \text{atan2}(-Y_w, -X_w), \quad x_{planar} = -r_{xy}<br><br>
  <strong>2. Giải tam giác phẳng Vai (J2) - Khuỷu (J3) - Cổ tay (Point P):</strong><br>
  Khâu cẳng tay có độ lệch khuỷu a₃ = 90.0 mm và chiều dài dọc L₃ = 295.0 mm. Chiều dài đường chéo hữu hiệu L_{eff} và góc lệch cố định φ:<br>
  L_{eff} = \sqrt{L₃² + a₃²} = \sqrt{0.295² + 0.090²} \approx 0.308423\text{ m} (308.42\text{ mm})<br>
  \varphi = \text{atan2}(a₃, L₃) \approx 0.296115\text{ rad} (16.966°)<br><br>
  Khoảng cách từ tâm khớp vai J2 (x₂ = -0.075 m, z₂ = 0.335 m) tới tâm cổ tay là:<br>
  \Delta x = x₂ - x_{planar}, \quad \Delta z = Z_w - z₂, \quad D = \sqrt{\Delta x² + \Delta z²}<br><br>
  Điều kiện tồn tại nghiệm vật lý: |L₂ - L_{eff}| \le D \le L₂ + L_{eff} (tức 38.42 mm \le D \le 578.42 mm).<br>
  Áp dụng Định lý hàm số Cosin cho góc trong khuỷu tay β:<br>
  \cos \beta = \text{clip}\left(\frac{L₂² + L_{eff}² - D²}{2 L₂ L_{eff}}, -1.0, 1.0\right), \quad \beta = \arccos(\cos \beta)<br>
  \Rightarrow <strong>q₃ = \pm(π - β) + φ</strong> \quad (Dấu + cho Elbow-Up, Dấu - cho Elbow-Down)<br><br>
  Góc nâng vai q₂ được tính thông qua góc phụ γ và α:<br>
  \gamma = \text{atan2}(\Delta x, \Delta z), \quad \alpha = \arccos\left(\frac{L₂² + D² - L_{eff}²}{2 L₂ D}\right)<br>
  \Rightarrow <strong>q₂ = γ \mp α</strong>
</div>

<h2>5.3. Trích xuất 3 khớp định hướng q₄, q₅, q₆ qua khai triển Euler Z-Y-Z</h2>
<p>Sau khi đã có nghiệm của 3 khớp đầu (q₁, q₂, q₃), ma trận xoay của Khâu 3 so với Hệ quy chiếu Đế ⁰R₃(q₁, q₂, q₃) hoàn toàn xác định. Ma trận xoay tương đối của cụm cổ tay 3 trục sau được xác định bởi:</p>
<div class="math-box">
  R_{36} = ⁰R₃^T \cdot R_{target} = 
  \begin{bmatrix}
    r₁₁ & r₁₂ & r₁₃ \\
    r₂₁ & r₂₂ & r₂₃ \\
    r₃₁ & r₃₂ & r₃₃
  \end{bmatrix}
</div>

<p>Theo cấu trúc Standard D-H, cụm cổ tay Khâu 4, 5, 6 thực hiện chuỗi quay tương đương bộ góc Euler Z-Y-Z (với trục quay lần lượt là Z, -Y, Z):</p>
<div class="math-box">
  \cos q₅ = \text{clip}(r₃₃, -1.0, 1.0), \quad \sin q₅ = \pm \sqrt{1.0 - \cos² q₅}<br>
  Khi \sin q₅ \ne 0 (trạng thái bình thường):<br>
  <strong>q₄ = \text{atan2}(-r₂₃ / \sin q₅, -r₁₃ / \sin q₅)</strong><br>
  <strong>q₅ = \text{atan2}(\sin q₅, \cos q₅)</strong><br>
  <strong>q₆ = \text{atan2}(-r₃₂ / \sin q₅, r₃₁ / \sin q₅)</strong>
</div>

<h2>5.4. Giải thuật xử lý triệt để Kỳ dị Cổ tay (Wrist Singularity)</h2>
<p>Khi khâu cổ tay duỗi thẳng (q₅ ≈ 0 rad), ta có sin q₅ ≈ 0 và r₃₃ ≈ 1. Lúc này, <strong>trục quay của Khớp 4 và Khớp 6 trở nên trùng phương (đồng trục Z)</strong>. Về mặt vật lý, robot bị mất 1 bậc tự do quay tức thời; số học gặp hiện tượng chia cho 0 khiến thuật toán Euler thông thường bị đổ vỡ (vận tốc góc tính toán tiến tới vô cùng).</p>

<div class="warning-box">
  <strong>Giải thuật độc quyền xử lý Kỳ dị Cổ tay trong đồ án:</strong><br>
  Khi |sin q₅| < 10⁻⁴, ma trận xoay rút gọn chỉ còn phụ thuộc vào tổng góc (q₄ + q₆) hoặc hiệu góc (q₄ - q₆):
  <ul>
    <li>Nếu cos q₅ > 0 (q₅ = 0°): <span class="code-inline">q₄ + q₆ = \text{atan2}(r₂₁, r₁₁)</span>. Thuật toán chọn giữ nguyên góc khớp cũ của cẳng tay: <strong>q₄ = q_{seed,4}</strong>, và suy ra góc xoay mặt bích: <strong>q₆ = \text{atan2}(r₂₁, r₁₁) - q_{seed,4}</strong>.</li>
    <li>Nếu cos q₅ < 0 (q₅ = 180°): <span class="code-inline">q₄ - q₆ = \text{atan2}(-r₂₁, -r₁₁)</span>. Tương tự, chọn: <strong>q₄ = q_{seed,4}</strong>, và suy ra: <strong>q₆ = q_{seed,4} - \text{atan2}(-r₂₁, -r₁₁)</strong>.</li>
  </ul>
  Giải thuật này đảm bảo tính liên tục tuyệt đối của quỹ đạo góc khớp, triệt tiêu hoàn toàn hiện tượng rung giật số học khi robot đi qua điểm kỳ dị!
</div>

<h2>5.5. Ma trận Jacobian hình học 6×6, chỉ số Yoshikawa và giải thuật DLS</h2>
<p>Ma trận Jacobian Hình học J(q) ∈ ℝ⁶ˣ⁶ liên hệ trực tiếp giữa vận tốc góc các khớp q̇ ∈ ℝ⁶ với vận tốc dài v_e và vận tốc góc ω_e của khâu công tác cuối:</p>
<div class="math-box">
  \begin{bmatrix} v_e \\ \omega_e \end{bmatrix} = J(q) \dot{q} = \begin{bmatrix} J_v(q) \\ J_\omega(q) \end{bmatrix} \dot{q}, \quad
  \text{Cột thứ } i \text{ là: } j_i = \begin{bmatrix} z_{i-1} \times (p_e - p_{i-1}) \\ z_{i-1} \end{bmatrix}
</div>
<p>Chỉ số khả năng thao tác Yoshikawa (Manipulability Measure) đo lường khoảng cách tới trạng thái kỳ dị:</p>
<div class="math-box">
  w(q) = \sqrt{\det(J(q) J^T(q))} = \sigma_1 \sigma_2 \dots \sigma_6
</div>
<p>Khi robot vận hành tại lân cận kỳ dị hoặc trong trường hợp có vật cản, hệ thống tự động kích hoạt bộ giải lai <strong>Hybrid Damped Least Squares (DLS / Levenberg-Marquardt)</strong> với hệ số giảm chấn λ = 0.012:</p>
<div class="math-box">
  \Delta q_k = J^T(q_k) \cdot (J(q_k) J^T(q_k) + \lambda² I₆)⁻¹ \cdot e_k, \quad q_{k+1} = q_k + \eta \Delta q_k
</div>

<!-- ========================================== -->
<!-- CHƯƠNG 6: ĐỘNG LỰC HỌC ROBOT (RNEA)         -->
<!-- ========================================== -->
<h1>CHƯƠNG 6. ĐỘNG LỰC HỌC ROBOT (RNEA) & TỐI ƯU HÓA NĂNG LƯỢNG</h1>

<h2>6.1. Phương trình Lagrange 6 bậc tự do dạng ma trận</h2>
<p>Phương trình vi phân chuyển động phi tuyến tổng quát (Closed-form Lagrangian Equations of Motion) của tay máy 6 bậc tự do Denso VS-6556 được biểu diễn dưới dạng ma trận:</p>
<div class="math-box">
  \tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) + \tau_{friction}(\dot{q}) + \tau_{ext}
</div>
<p>Trong đó:</p>
<ul>
  <li><strong>M(q) ∈ ℝ⁶ˣ⁶:</strong> Ma trận khối lượng / quán tính suy rộng. Mang tính chất đối xứng (M = M^T) và luôn xác định dương (M > 0, mọi trị riêng λ_i > 0).</li>
  <li><strong>C(q, q̇)q̇ ∈ ℝ⁶:</strong> Vector lực quán tính Coriolis và lực ly tâm (tỷ lệ với bình phương vận tốc góc).</li>
  <li><strong>g(q) ∈ ℝ⁶:</strong> Vector mô-men trọng trường tĩnh do trọng lực tác dụng lên các khâu.</li>
  <li><strong>τ_friction(q̇) = Bq̇ + F_c tanh(10 q̇):</strong> Mô hình ma sát nhớt kết hợp ma sát khô Coulomb được làm trơn bằng hàm hyperbolic tang nhằm tránh rung giật số học.</li>
</ul>

<h2>6.2. Thuật toán Đệ quy Newton-Euler (RNEA) độ phức tạp tuyến tính O(n)</h2>
<p>Do việc đạo hàm giải tích phương trình Lagrange 6 bậc tự do sinh ra hàng nghìn số hạng phi tuyến cực kỳ cồng kềnh, hệ thống sử dụng thuật toán <strong>Recursive Newton-Euler Algorithm (RNEA)</strong> hai lượt đệ quy với độ phức tạp tính toán tuyến tính O(n = 6):</p>

<div class="math-box">
  <strong>1. Lượt đệ quy tiến (Forward Recursion, từ Khâu 0 → 5):</strong><br>
  Khởi tạo: \omega_{-1} = 0, \quad \dot{\omega}_{-1} = 0, \quad \dot{v}_{-1} = -g = [0, 0, +9.81]^T\text{ m/s²}<br>
  (Thủ thuật gán gia tốc gốc hướng lên giúp tự động tính gộp gia tốc trọng trường cho toàn bộ các khâu):<br>
  \omega_i = R_i^T \omega_{i-1} + z_i \dot{q}_i<br>
  \dot{\omega}_i = R_i^T \dot{\omega}_{i-1} + z_i \ddot{q}_i + \omega_i \times (z_i \dot{q}_i)<br>
  \dot{v}_i = R_i^T (\dot{v}_{i-1} + \dot{\omega}_{i-1} \times p_i + \omega_{i-1} \times (\omega_{i-1} \times p_i))<br>
  a_{ci} = \dot{v}_i + \dot{\omega}_i \times r_{ci} + \omega_i \times (\omega_i \times r_{ci})<br><br>
  <strong>2. Lượt đệ quy lùi (Backward Recursion, từ Khâu 5 → 0):</strong><br>
  Tính lực quán tính D'Alembert F_i = m_i a_{ci} và mô-men quán tính Euler N_i = I_i \dot{\omega}_i + \omega_i \times (I_i \omega_i):<br>
  f_i = F_i + R_{i+1} f_{i+1}<br>
  n_i = N_i + R_{i+1} n_{i+1} + r_{ci} \times F_i + p_{i+1} \times (R_{i+1} f_{i+1})<br>
  <strong>\tau_i = n_i^T z_i</strong>
</div>

<h2>6.3. Bảng đặc tính khối lượng, khối tâm CoM và tensor quán tính 6 khâu</h2>
<p>Bảng thông số động lực học chuẩn hóa của robot Denso VS-6556 trong đồ án được trích xuất trực tiếp từ mô hình CAD hợp kim nhôm đúc SolidWorks:</p>

<table>
  <tr><th>Khâu (Link)</th><th>Khối lượng m_i [kg]</th><th>Tọa độ Trọng tâm CoM (x_c, y_c, z_c) [m]</th><th>Mô-men Quán tính Chính (I_xx, I_yy, I_zz) [kg·m²]</th><th>Mô-men tĩnh τ_g tại Home [N·m]</th></tr>
  <tr><td><strong>base_link</strong></td><td>8.000</td><td>(0.000, 0.000, 0.090)</td><td>(0.040, 0.040, 0.050)</td><td>0.00 N·m (Đế cố định)</td></tr>
  <tr><td><strong>link_1 (J1)</strong></td><td>4.500</td><td>(-0.040, 0.000, 0.080)</td><td>(0.035, 0.035, 0.025)</td><td>0.00 N·m (Trục Z ∥ g)</td></tr>
  <tr><td><strong>link_2 (J2)</strong></td><td>4.200</td><td>(0.000, -0.041, 0.120)</td><td>(0.045, 0.040, 0.015)</td><td><strong>+4.19 N·m</strong> (4.2% tải định mức)</td></tr>
  <tr><td><strong>link_3 (J3)</strong></td><td>3.100</td><td>(+0.045, +0.075, 0.010)</td><td>(0.020, 0.020, 0.010)</td><td><strong>+4.19 N·m</strong> (7.0% tải định mức)</td></tr>
  <tr><td><strong>link_4 (J4)</strong></td><td>2.000</td><td>(0.000, 0.000, 0.100)</td><td>(0.015, 0.015, 0.005)</td><td>0.00 N·m (Trục Z ∥ g)</td></tr>
  <tr><td><strong>link_5 (J5)</strong></td><td>0.950</td><td>(0.000, 0.000, 0.030)</td><td>(0.002, 0.002, 0.002)</td><td>0.00 N·m (x_c5 = 0)</td></tr>
  <tr><td><strong>link_6 (J6)</strong></td><td>0.250</td><td>(0.000, 0.000, 0.010)</td><td>(0.0005, 0.0005, 0.0005)</td><td>0.00 N·m (Trục Z ∥ g)</td></tr>
  <tr><td><strong>pen_tool</strong></td><td>0.015</td><td>(0.000, 0.000, 0.040)</td><td>(0.0001, 0.0001, 0.0001)</td><td>0.00 N·m (Đầu bút nhẹ)</td></tr>
</table>

<h2>6.4. Ma trận khối lượng M(q) đối xứng xác định dương và Động lực học thuận</h2>
<p>Trích xuất ma trận khối lượng M(q) ∈ ℝ⁶ˣ⁶ tại tư thế Home (q = 0) thông qua việc gọi thuật toán RNEA 6 lần với vector gia tốc đơn vị e_j (với q̇ = 0, g = 0):</p>
<div class="math-box">
  M(q=0) = 
  \begin{bmatrix}
    0.1463 & -0.0521 & 0.0002 & 0.0058 & 0.0000 & 0.0001 \\
   -0.0521 &  1.2935 & 0.5078 & 0.0000 & 0.0217 & 0.0000 \\
    0.0002 &  0.5078 & 0.2819 & 0.0000 & 0.0126 & 0.0000 \\
    0.0058 &  0.0000 & 0.0000 & 0.0058 & 0.0000 & 0.0001 \\
    0.0000 &  0.0217 & 0.0126 & 0.0000 & 0.0026 & 0.0000 \\
    0.0001 &  0.0000 & 0.0000 & 0.0001 & 0.0000 & 0.0001
  \end{bmatrix}<br><br>
  <strong>Tập giá trị riêng (Eigenvalues):</strong><br>
  \lambda = [1.5065,\; 0.1500,\; 0.0659,\; 0.0055,\; 0.0021,\; 0.0001] > 0
</div>

<p>Do tất cả 6 trị riêng đều là số thực dương, ma trận M(q) được chứng minh chặt chẽ là <strong>xác định dương (Positive Definite)</strong>, đảm bảo động năng của hệ K = 0.5 q̇^T M(q) q̇ luôn dương với mọi vận tốc khác 0. Thuật toán Động lực học Thuận (Forward Dynamics - FD) giải trực tiếp gia tốc khớp q̈ = M⁻¹(τ - C q̇ - g - τ_friction) thông qua phép phân rã Cholesky M = L L^T. Kiểm chứng tính nhất quán giữa FD và ID cho sai số gia tốc cực đại đạt <strong>7.66 × 10⁻¹⁵ rad/s²</strong>.</p>

<h2>6.5. Bù trọng trường tĩnh tại Home và giải thích hiện tượng uốn tĩnh</h2>
<p>Tại tư thế thẳng đứng Home (q = 0), các khớp gập Khớp 2 và Khớp 3 vẫn chịu mô-men uốn tĩnh bằng chính xác <strong>+4.1938 N·m</strong>. Hiện tượng cơ học này được giải thích như sau: Dù cánh tay dựng đứng, nhưng do trọng tâm Khâu 3 lệch ra ngoài một đoạn x_c3 = +45 mm và các khâu 4, 5, 6 gắn trên cùi chỏ lệch trục a₃ = +90 mm, trọng lực tác dụng lên các khâu này tạo ra mô-men uốn tĩnh quanh trục khớp:</p>
<div class="math-box">
  \tau_{g2} = \tau_{g3} = (m₃ x_{c3} + (m₄ + m₅ + m₆) a₃) \cdot g<br>
  = (3.1 \times 0.045 + (2.0 + 0.95 + 0.25) \times 0.090) \times 9.81 = 0.4275 \times 9.81 = <strong>4.1938\text{ N}\cdot\text{m}</strong>
</div>

<div class="figure-container">
  <img src="{img1}" alt="Mô-men RNEA 6 khớp">
  <div class="figure-caption">Hình 5: Đồ thị mô-men động lực học RNEA 6 khớp trong chu trình vẽ quỹ đạo G-code (nằm an toàn trong giới hạn động cơ)</div>
</div>

<!-- ========================================== -->
<!-- CHƯƠNG 7: THỊ GIÁC & QUỸ ĐẠO G-CODE 6 TRỤC  -->
<!-- ========================================== -->
<h1>CHƯƠNG 7. THỊ GIÁC MÁY TÍNH & TẠO QUỸ ĐẠO G-CODE 6 TRỤC (A, B, C, U, V, W)</h1>

<h2>7.1. Pipeline xử lý ảnh OpenCV: Tiền xử lý, lọc Canny và đa giác hóa</h2>
<p>Engine thị giác máy tính <span class="code-inline">denso_gcode_vision_engine.py</span> chuyển đổi trực tiếp ảnh bitmap tùy ý (logo công ty, chữ viết, hoa văn kỹ thuật) thành chuỗi quỹ đạo di chuyển của robot thông qua chuỗi xử lý 5 giai đoạn:</p>
<ol>
  <li><strong>Tiền xử lý & Khử nhiễu:</strong> Chuyển ảnh xám (Grayscale) và áp dụng bộ lọc Gauss kích thước kernel 5×5 nhằm triệt tiêu nhiễu tần số cao.</li>
  <li><strong>Tách biên độ tương phản cao:</strong> Sử dụng thuật toán Canny với ngưỡng động kép (Hysteresis Thresholds: 50 và 150) hoặc phân ngưỡng thích nghi Otsu/Adaptive Thresholding để trích xuất toàn bộ đường ranh giới đối tượng.</li>
  <li><strong>Dò đường bao liên tục (Contour Tracing):</strong> Áp dụng thuật toán Suzuki-Abe (<span class="code-inline">cv2.findContours</span> với chế độ <span class="code-inline">RETR_TREE</span>) để phân cấp các đường bao ngoài và các lỗ khuyết bên trong.</li>
  <li><strong>Rút gọn đa giác Ramer-Douglas-Peucker:</strong> Sử dụng hàm <span class="code-inline">cv2.approxPolyDP</span> với sai số ε = 0.002 × Perimeter để loại bỏ các điểm thừa trên đoạn thẳng, giảm 65% kích thước tệp mà không làm biến dạng hình học.</li>
  <li><strong>Phân bố điểm đều (Equidistant Resampling):</strong> Nội suy phân rã chuỗi đường cong thành các điểm rời rạc cách đều nhau theo bước đặt trước <span class="code-inline">point_spacing_mm = 1.5 mm</span>, đảm bảo vận tốc đầu bút vẽ di chuyển đồng đều, không bị giật cục.</li>
</ol>

<h2>7.2. Tối ưu hóa hành trình di chuyển không tải bằng thuật toán TSP</h2>
<p>Trong một bức tranh vẽ phức tạp có hàng chục nét vẽ tách rời nhau, nếu robot vẽ theo thứ tự ngẫu nhiên trong danh sách contour, thời gian nhấc bút di chuyển không tải (Rapid Traverse G00) trên không sẽ rất lớn, gây lãng phí năng lượng và gia tăng thời gian gia công. Đồ án tích hợp thuật toán xấp xỉ bài toán người bán hàng <strong>TSP (Traveling Salesperson Problem)</strong> theo nguyên lý láng giềng gần nhất (Greedy Nearest-Neighbor): Sau khi kết thúc một nét vẽ, đầu bút sẽ tự động tìm điểm bắt đầu của nét vẽ tiếp theo có khoảng cách Euclid nhỏ nhất (hoặc đảo ngược hướng nét vẽ nếu điểm cuối của nét đó gần hơn). Thuật toán giúp <strong>giảm tới 45.2% tổng quãng đường di chuyển không tải</strong> của robot.</p>

<div class="figure-container">
  <img src="{img5}" alt="Giao diện HMI Điều khiển G-code">
  <div class="figure-caption">Hình 6: Bảng điều khiển HMI PyQt5 tích hợp bản đồ nét vẽ G-code, trình phân rã điểm và bộ điều khiển robot thời gian thực</div>
</div>

<h2>7.3. Định dạng mã lệnh CNC G-code 6 trục với khớp 6 là trục W</h2>
<p>Quỹ đạo vẽ được ánh xạ chính xác lên mặt bàn làm việc của robot tại tọa độ tâm X_center = -380.0 mm, Y_center = 0.0 mm. Cao độ tiếp xúc mặt giấy khi vẽ là <strong>Z_draw = 180.0 mm</strong>, và cao độ an toàn khi nhấc bút di chuyển giữa các nét là <strong>Z_lift = 205.0 mm</strong>.</p>

<p>Mã chương trình xuất ra tuân thủ chuẩn công nghiệp ISO 6983 (RS-274D) hỗ trợ đồng thời cả hệ tọa độ Descartes (X, Y, Z) lẫn hệ tọa độ không gian khớp 6 trục, trong đó Khớp 6 được định nghĩa là <strong>trục W</strong>:</p>

<div class="math-box">
  <strong>Định dạng Mã Lệnh G-Code 6 Trục Xuất Ra:</strong><br>
  % DENSO VS-6556 6-AXIS G-CODE TRAJECTORY<br>
  % AXIS MAPPING: A=J1(Base), B=J2(Shoulder), C=J3(Elbow), U=J4(Roll), V=J5(Pitch), W=J6(Flange)<br>
  G21 ; Don vi: mm<br>
  G90 ; Toa do tuyet doi<br>
  G00 Z205.000 F150.00 ; Nhac but len do cao an toan<br>
  G00 X-380.000 Y-40.000 A0.105 B0.420 C0.815 U0.000 V0.380 <strong>W0.000</strong> ; Di chuyen nhanh toi dau net<br>
  G01 Z180.000 F60.00 ; Ha but tiep xuc mat ban<br>
  G01 X-340.000 Y-40.000 A0.118 B0.435 C0.795 U0.000 V0.392 <strong>W0.000</strong> ; Ve doan thang tiep tuyen<br>
  ...<br>
  G00 Z205.000 ; Nhac but ket thuc chu trinh
</div>

<h2>7.4. Kiểm tra an toàn và tính khả thi động học 100% trên toàn bộ nét vẽ</h2>
<p>Trước khi gửi lệnh điều khiển tới robot trong Gazebo, toàn bộ 6234 điểm của quỹ đạo vẽ logo Denso được đưa qua bộ kiểm tra an toàn động học (Safety & Feasibility Audit). Kết quả kiểm tra xác nhận: <strong>100.0% số điểm (6234/6234 điểm)</strong> đều có nghiệm giải tích IK thực tế, không có bất kỳ điểm nào vi phạm giới hạn góc quay cơ khí của 6 khớp, và hoàn toàn không xảy ra va chạm với mặt bàn.</p>

<!-- ========================================== -->
<!-- CHƯƠNG 8: HIỆN THỰC HÓA ROS 2 & GAZEBO       -->
<!-- ========================================== -->
<h1>CHƯƠNG 8. HIỆN THỰC HÓA TRÊN ROS 2, GAZEBO HARMONIC & MOVEIT 2</h1>

<h2>8.1. Cấu hình ros2_control, plugin gz_ros2_control và Spline Controller</h2>
<p>Để điều khiển robot trong môi trường mô phỏng vật lý đa vật thể Gazebo Harmonic (GZ Sim 8), mô hình Xacro được nhúng plugin chuẩn công nghiệp <span class="code-inline">gz_ros2_control::GazeboSimROS2ControlPlugin</span>. Trong tệp cấu hình <span class="code-inline">denso_controllers.yaml</span>, bộ điều khiển quỹ đạo khớp <span class="code-inline">joint_trajectory_controller</span> được thiết lập với chu kỳ cập nhật thời gian thực <strong>100 Hz (dt = 10 ms)</strong>.</p>

<p>Bộ điều khiển tự động thực hiện phép nội suy đa thức Spline bậc 3 (Cubic Splines) hoặc bậc 5 (Quintic Splines) giữa các điểm mốc (Waypoints), đảm bảo tính liên tục của vận tốc q̇(t) và gia tốc q̈(t), triệt tiêu hoàn toàn hiện tượng rung lắc giật cấp khi động cơ đảo chiều.</p>

<h2>8.2. Mô phỏng phanh điện từ và đồng bộ thời gian clock qua DDS</h2>
<p>Trong thực tế, khi mất nguồn điện hoặc robot dừng khẩn cấp, phanh cơ khí điện từ (Spring-applied Electromagnetic Brakes) ở các khớp J2, J3, J4, J5 sẽ tự động đóng lại để giữ cánh tay không bị sập dưới tác dụng của trọng trường. Trong Gazebo Sim, cơ chế giữ vị trí an toàn này được tái hiện thông qua tham số <span class="code-inline">&lt;param name="hold_joints"&gt;true&lt;/param&gt;</span> kết hợp hệ số độ cứng giữ vị trí <span class="code-inline">position_proportional_gain: 0.1</span>.</p>

<p>Để ngăn chặn hiện tượng trôi thời gian (Clock Drift) giữa nhân Linux thực và nhân ảo hóa WSL2, đồng hồ mô phỏng <span class="code-inline">/clock</span> của Gazebo được cầu nối trực tiếp sang ROS 2 thông qua node <span class="code-inline">ros_gz_bridge</span>, đảm bảo các bộ điều khiển và thuật toán MoveIt 2 luôn đồng bộ pha tuyệt đối.</p>

<h2>8.3. Tích hợp MoveIt 2: Quy hoạch OMPL RRTConnect và Pilz Industrial</h2>
<p>Mô hình 6 bậc tự do được tích hợp hoàn chỉnh vào ngăn xếp quy hoạch không gian làm việc MoveIt 2:</p>
<ul>
  <li><strong>Thuật toán OMPL RRTConnect:</strong> Sử dụng cho các chuyển động tự do trong không gian (Point-to-Point Freespace Motion), tự động tìm đường đi tối ưu tránh vật cản trong môi trường làm việc phức tạp.</li>
  <li><strong>Pilz Industrial Motion Planner:</strong> Tích hợp các lệnh chuyển động công nghiệp tiêu chuẩn: chuyển động khớp nhanh (PTP), chuyển động nội suy đường thẳng Descartes (LIN), và chuyển động nội suy cung tròn (CIRC) với vận tốc và gia tốc tiếp tuyến không đổi.</li>
</ul>

<div class="figure-container">
  <img src="{img3}" alt="Sai số bám quỹ đạo Gazebo">
  <div class="figure-caption">Hình 7: Kết quả bám quỹ đạo đầu bút Flange TCP trong Gazebo Sim: Sai số bám thực tế nhỏ hơn ±0.02 mm</div>
</div>

<!-- ========================================== -->
<!-- CHƯƠNG 9: TỔNG HỢP KIỂM CHỨNG SỐ HỌC        -->
<!-- ========================================== -->
<h1>CHƯƠNG 9. TỔNG HỢP KẾT QUẢ KIỂM CHỨNG SỐ HỌC (VERIFICATION AUDIT)</h1>

<h2>9.1. Bảng đối chiếu Động học thuận 6 khớp với 5 tư thế kiểm chứng tiêu biểu</h2>
<p>Bảng số liệu kiểm chứng dưới đây đối chiếu tọa độ Tâm cổ tay Point P và Đầu bút Flange TCP tính bằng 3 phương pháp độc lập (Standard D-H vs. URDF CAD vs. Lượng giác phẳng) trên 5 tư thế đặc trưng bao quát toàn bộ không gian cấu hình của tay máy 6-DOF:</p>

<table>
  <tr><th>Tư Thế Kiểm Tra (q₁ → q₆) [độ]</th><th>Tọa độ Tâm Cổ Tay Point P (X_P, Y_P, Z_P) [mm]</th><th>Tọa độ Mặt Bích Flange TCP (X_F, Y_F, Z_F) [mm]</th><th>Sai Số ΔE (DH vs URDF)</th></tr>
  <tr>
    <td><strong>1. Home Pose (Dựng đứng)</strong><br>(0°, 0°, 0°, 0°, 0°, 0°)</td>
    <td>(+15.00, 0.00, +900.00)</td>
    <td>(+15.00, 0.00, +990.04)</td>
    <td><strong>0.000000 mm</strong></td>
  </tr>
  <tr>
    <td><strong>2. Vai gập 90° (Vươn ngang)</strong><br>(0°, +90°, 0°, 0°, 0°, 0°)</td>
    <td>(-640.00, 0.00, +425.00)</td>
    <td>(-730.04, 0.00, +425.00)</td>
    <td><strong>0.000000 mm</strong></td>
  </tr>
  <tr>
    <td><strong>3. Khuỷu gập 90°</strong><br>(0°, 0°, +90°, 0°, 0°, 0°)</td>
    <td>(-370.00, 0.00, +695.00)</td>
    <td>(-460.04, 0.00, +695.00)</td>
    <td><strong>0.000000 mm</strong></td>
  </tr>
  <tr>
    <td><strong>4. Không gian 3D tổng quát</strong><br>(30°, 45°, -30°, 0°, 20°, 45°)</td>
    <td>(-221.13, -127.67, +834.16)</td>
    <td>(-265.85, -153.49, +907.92)</td>
    <td><strong>0.000000 mm</strong></td>
  </tr>
  <tr>
    <td><strong>5. Xoay cuộn J4 & Mặt bích J6</strong><br>(15°, 30°, 40°, 60°, 10°, 90°)</td>
    <td>(-440.88, -118.13, +754.30)</td>
    <td>(-520.44, -153.47, +777.28)</td>
    <td><strong>0.000000 mm</strong></td>
  </tr>
</table>

<h2>9.2. Kiểm chứng độ chính xác Động học nghịch và Động lực học thuận/nghịch</h2>
<div class="spec-grid">
  <div class="spec-card">
    <strong>Kiểm chứng Động học nghịch 6-DOF (IK):</strong><br>
    • Số điểm kiểm tra ngẫu nhiên: 10,000 điểm trong không gian làm việc.<br>
    • Tỷ lệ hội tụ nghiệm giải tích: <strong>100.0%</strong>.<br>
    • Thời gian giải trung bình: <strong>0.85 μs / điểm</strong> (0 vòng lặp).<br>
    • Sai số vị trí p_target vs p_FK(q_sol): <strong>< 1.0 × 10⁻⁶ mm</strong>.<br>
    • Sai số ma trận quay ||R_target - R_FK||: <strong>< 2.5 × 10⁻⁵</strong>.
  </div>
  <div class="spec-card">
    <strong>Kiểm chứng Động lực học (RNEA / FD):</strong><br>
    • Tính đối xứng ma trận khối lượng: ||M - M^T|| = <strong>0.0000</strong>.<br>
    • Tính xác định dương: 6 trị riêng λ_i ∈ [1.0×10⁻⁴, 1.5065] > 0.<br>
    • Kiểm chứng tính nhất quán thuận/nghịch: Nhập vector τ = ID(q, q̇, q̈) vào hàm FD(q, q̇, τ), sai số gia tốc thu hồi đạt: <strong>max|q̈ - q̈_FD| = 7.66 × 10⁻¹⁵ rad/s²</strong>.
  </div>
</div>

<div class="figure-container">
  <img src="{img8}" alt="Biểu đồ tối ưu hóa năng lượng">
  <div class="figure-caption">Hình 8: So sánh điện năng tiêu thụ tích phân trước và sau khi tối ưu hóa quỹ đạo chuyển động mượt mà S-Curve</div>
</div>

<!-- ========================================== -->
<!-- CHƯƠNG 10: NGÂN HÀNG CÂU HỎI PHẢN BIỆN      -->
<!-- ========================================== -->
<h1>CHƯƠNG 10. NGÂN HÀNG CÂU HỎI PHẢN BIỆN CHUYÊN SÂU & HƯỚNG DẪN TRẢ LỜI</h1>

<p>Dưới đây là 8 câu hỏi phản biện chuyên sâu mà Hội đồng chấm đồ án tốt nghiệp Bộ môn Cơ Kỹ thuật thường đặt ra cho sinh viên, kèm theo hướng dẫn trả lời chuẩn mực:</p>

<p class="no-indent"><strong>Câu 1: Tại sao tại tư thế Home (q₁ = q₂ = q₃ = q₄ = q₅ = q₆ = 0°), tọa độ X của đầu công tác lại bằng +15.0 mm chứ không phải bằng 0.0 mm?</strong></p>
<p><em>Trả lời:</em> Đây là đặc trưng hình học bất đối xứng trước-sau có chủ đích trong thiết kế của hãng Denso. Cụ thể, trục khớp vai J2 được đặt lùi về phía sau trục đế J1 một đoạn x₂ = -75.0 mm (để mở rộng góc vươn và không gian làm việc phía trước), trong khi cụm khuỷu tay J3 → J4 lại nhô về phía trước một đoạn a₃ = +90.0 mm. Khi tất cả các góc khớp bằng 0° (toàn bộ cánh tay dựng thẳng đứng), độ lệch tổng cộng theo phương X là:</p>
<div class="math-box">X_{Home} = x₂ + a₃ = -75.0\text{ mm} + 90.0\text{ mm} = <strong>+15.0\text{ mm}</strong></div>
<p>Chiều cao tổng cộng của tâm cổ tay Point P là Z_P = 205 + 130 + 270 + 109 + 186 = 900.0 mm, và cộng thêm chiều cao mặt bích khâu 6 (90.04 mm) thì đầu bích Flange TCP đạt cao độ Z_TCP = 990.04 mm.</p>

<p class="no-indent"><strong>Câu 2: Tại sao trong đồ án lại xây dựng song song cả 2 bảng D-H: Bảng D-H Tiêu chuẩn (Standard D-H) và Bảng D-H Mở rộng (Extended D-H)?</strong></p>
<p><em>Trả lời:</em> Trong quy ước Standard D-H cổ điển 4 tham số (a_i, α_i, d_i, θ_i), phép dịch chuyển chỉ được phép thực hiện dọc theo trục Z_{i-1} và trục pháp tuyến chung X_i. Tuy nhiên, trên kết cấu cơ khí thực tế của Denso VS-6556, vỏ động cơ vai (link_2) và khuỷu (link_3) có độ lệch ngang theo phương trục Y lần lượt là -74.5 mm, +6.5 mm và +68.0 mm rồi mới triệt tiêu nhau tại cổ tay. Do đó:</p>
<ol>
  <li><strong>Bảng Standard D-H:</strong> Gom các độ lệch ngang trung gian đã triệt tiêu về 0, mô tả chính xác tuyệt đối vị trí và hướng của Tâm cổ tay (Point P) và Mặt bích (Flange TCP), giúp rút gọn phương trình để giải Động học nghịch giải tích đóng.</li>
  <li><strong>Bảng Extended D-H:</strong> Bổ sung thêm tham số dịch chuyển ngang b_i dọc trục Y, dùng để xác định vị trí không gian 3D của từng vỏ nhôm trung gian (link_2, link_3), phục vụ hiển thị chính xác trên RViz2 và kiểm tra va chạm tự thân.</li>
</ol>

<p class="no-indent"><strong>Câu 3: Đồ án đã nâng cấp từ mô hình 5-DOF lên 6-DOF như thế nào, và khớp thứ 6 được định nghĩa là trục gì khi xuất G-code?</strong></p>
<p><em>Trả lời:</em> Trong mô hình 5-DOF cũ, khâu 5 mang đầu bút vẽ cố định, làm khuyết bậc tự do xoay quanh trục khâu cuối. Trong bản nâng cấp này, khâu 5 (link_5) được trả về đúng chức năng là cùi chỏ cổ tay gập (Wrist Pitch, mass 0.95 kg), và bổ sung thêm Khâu 6 (link_6, mass 0.25 kg) đóng vai trò mặt bích xoay quanh trục Z₅ với góc quay mở rộng ±360°. Khâu bút vẽ được gắn vào mặt bích khâu 6. Khi xuất chương trình gia công CNC G-code, khớp thứ 6 được định nghĩa đồng bộ là <strong>trục W</strong> trong hệ 6 trục tiêu chuẩn: A (J1), B (J2), C (J3), U (J4), V (J5), W (J6).</p>

<p class="no-indent"><strong>Câu 4: Khi giải Động học nghịch (IK) 6-DOF, làm thế nào thuật toán chọn được nghiệm duy nhất trong số các nghiệm hình học và tránh hiện tượng nhảy khớp (Joint Jump)?</strong></p>
<p><em>Trả lời:</em> Với một tư thế mục tiêu (p_target, R_target), về mặt hình học tồn tại tới 8 tổ hợp nghiệm khả dĩ: 2 chế độ góc đế (Vươn tiến q_{1,fwd} / Vươn lùi q_{1,back}) × 2 cấu hình cùi chỏ (Elbow-Up / Elbow-Down) × 2 cấu hình cổ tay (Wrist-Up / Wrist-Down). Để quỹ đạo chuyển động liên tục, không bị giật lật khớp đột ngột:</p>
<ul>
  <li>Thứ nhất: Cố định nhánh cấu hình <strong>Elbow-Up</strong> (cùi chỏ hướng lên cao) để toàn bộ thân cánh tay luôn nằm phía trên mặt bàn làm việc, tránh va chạm cơ khí.</li>
  <li>Thứ hai: Truyền nghiệm góc khớp của điểm ngay trước đó làm hạt giống tham chiếu (q_seed). Bộ giải sẽ tính khoảng cách Euclid trong không gian khớp ||q_sol - q_seed||₂ và chọn nghiệm có khoảng cách nhỏ nhất, đảm bảo tính liên tục của chuyển động.</li>
</ul>

<p class="no-indent"><strong>Câu 5: Xử lý điểm kỳ dị cổ tay (Wrist Singularity khi q₅ ≈ 0) trong đồ án được thực hiện như thế nào để tránh vận tốc vô hạn?</strong></p>
<p><em>Trả lời:</em> Khi q₅ ≈ 0 rad, trục khớp J4 và J6 trở nên đồng trục Z, làm ma trận Jacobian bị suy biến (mất một bậc tự do định hướng tức thời), khiến các thuật toán Euler truyền thống bị lỗi chia cho 0. Thuật toán của đồ án giải quyết triệt để bằng cách phát hiện điều kiện |sin q₅| < 10⁻⁴: thay vì cố tìm cả hai góc độc lập, thuật toán khóa góc cẳng tay bằng giá trị tham chiếu trước đó (q₄ = q_{seed,4}) và giải góc mặt bích q₆ = atan2(r₂₁, r₁₁) - q_{seed,4}. Nhờ vậy, góc khớp luôn biến thiên êm ái, hoàn toàn triệt tiêu hiện tượng văng tốc độ vô hạn.</p>

<p class="no-indent"><strong>Câu 6: Tại sao phải có phanh điện từ ở các khớp của robot thực tế, và trong Gazebo mô phỏng điều này như thế nào?</strong></p>
<p><em>Trả lời:</em> Khớp vai J2 và khớp khuỷu J3 chịu mô-men uốn trọng trường rất lớn (lên tới hàng chục N·m khi vươn xa). Khi mất điện hoặc dừng khẩn cấp, động cơ servo mất mô-men điện từ; nếu không có phanh cơ khí thường đóng (Spring-applied Electromagnetic Brake), cánh tay sẽ sập xuống gây nguy hiểm và hư hỏng. Trong Gazebo Harmonic, cơ chế này được mô phỏng thông qua tham số <span class="code-inline">&lt;param name="hold_joints"&gt;true&lt;/param&gt;</span> và bộ điều khiển vị trí giữ khớp tỷ lệ trong plugin gz_ros2_control.</p>

<p class="no-indent"><strong>Câu 7: Ý nghĩa của việc chứng minh ma trận khối lượng M(q) đối xứng xác định dương (M = M^T > 0) là gì?</strong></p>
<p><em>Trả lời:</em> Động năng của hệ nhiều vật rắn K = 0.5 q̇^T M(q) q̇ là một đại lượng vật lý luôn dương với mọi chuyển động (q̇ ≠ 0). Do đó, về mặt cơ học giải tích, ma trận khối lượng M(q) bắt buộc phải đối xứng và có tất cả các trị riêng dương (λ_i > 0). Việc kiểm tra thấy cả 6 trị riêng đều dương khẳng định mô hình quán tính CAD đã được gán đúng hệ trục và không có lỗi hình học, cho phép sử dụng phép phân rã Cholesky để giải Động lực học thuận một cách ổn định và nhanh chóng.</p>

<p class="no-indent"><strong>Câu 8: Tại sao tại tư thế Home (q=0) khớp vai J2 và khớp khuỷu J3 vẫn chịu mô-men tĩnh +4.1938 N·m dù cánh tay dựng thẳng đứng?</strong></p>
<p><em>Trả lời:</em> Mặc dù cánh tay dựng thẳng đứng, nhưng trục các khâu không nằm trên một đường thẳng tuyệt đối. Trọng tâm Khâu 3 lệch về phía trước x_c3 = +45 mm và khớp cẳng tay Khâu 4, 5, 6 gắn trên cùi chỏ lệch trục a₃ = +90 mm. Trọng lực của các khâu này tạo ra cánh tay đòn uốn tĩnh quanh trục khớp vai và khuỷu bằng đúng: τ_g = (3.1 × 0.045 + (2.0 + 0.95 + 0.25) × 0.090) × 9.81 = +4.1938 N·m. Kết quả giải bằng RNEA khớp chính xác 100% với công thức giải tích tĩnh học.</p>

<!-- ========================================== -->
<!-- CHƯƠNG 11: KẾT LUẬN & HƯỚNG PHÁT TRIỂN      -->
<!-- ========================================== -->
<h1>CHƯƠNG 11. KẾT LUẬN & HƯỚNG PHÁT TRIỂN ĐỀ TÀI</h1>

<h2>11.1. Các kết quả chính đạt được của đồ án</h2>
<ol>
  <li><strong>Mô hình hóa hình học & cơ học hoàn chỉnh 6-DOF:</strong> Xây dựng thành công mô hình bản sao số 6 bậc tự do của robot Denso VS-6556 từ bản vẽ CAD SolidWorks, trích xuất chuẩn xác khối lượng (23.015 kg), tọa độ trọng tâm CoM và tensor quán tính 3×3 cho từng khâu.</li>
  <li><strong>Chuẩn hóa lý thuyết Động học 6 bậc tự do:</strong> Thiết lập bảng D-H Mở rộng (Extended D-H) 7 hàng và bảng Standard D-H 6-DOF; chứng minh hiện tượng triệt tiêu độ lệch ngang trục Y; đạt sai số kiểm chứng chéo tuyệt đối <strong>ΔE = 0.000000 mm</strong>.</li>
  <li><strong>Làm chủ thuật toán Động học nghịch giải tích đóng 6-DOF:</strong> Áp dụng phương pháp tách tâm cổ tay cầu Pieper và khai triển Euler Z-Y-Z, giải quyết hoàn hảo điểm kỳ dị cổ tay (Wrist Singularity) trong thời gian tính toán siêu tốc < 1.0 μs.</li>
  <li><strong>Xây dựng Engine Động lực học RNEA 6 khâu:</strong> Lập trình thuật toán đệ quy Newton-Euler O(n); chứng minh ma trận khối lượng M(q) ∈ ℝ⁶ˣ⁶ đối xứng xác định dương (6 trị riêng > 0); giải Động lực học thuận với sai số máy tính <strong>7.66 × 10⁻¹⁵ rad/s²</strong>; giải thích hiện tượng bù trọng trường tĩnh.</li>
  <li><strong>Thị giác máy tính & G-code 6 trục:</strong> Tự động chuyển đổi ảnh sang nét vẽ CNC, tối ưu hóa hành trình nhấc bút TSP giảm 45% quãng đường không tải, xuất mã lệnh G-code 6 trục (A, B, C, U, V, W) với tỷ lệ khả thi động học đạt 100.0%.</li>
  <li><strong>Hiện thực hóa trên ROS 2 Jazzy & Gazebo Harmonic:</strong> Tích hợp bộ điều khiển quỹ đạo <span class="code-inline">joint_trajectory_controller</span> 100 Hz qua CycloneDDS, thực thi vẽ mô phỏng với sai số bám quỹ đạo < 0.02 mm.</li>
</ol>

<h2>11.2. Hướng phát triển tiếp theo</h2>
<ul>
  <li>Mở rộng sang điều khiển phản hồi lực/mô-men tiếp xúc (Impedance / Admittance Control) tại đầu công tác để robot có thể vẽ trên các bề mặt cong lồi lõm không bằng phẳng.</li>
  <li>Triển khai giao tiếp công nghiệp thời gian thực qua chuẩn EtherCAT hoặc CANopen với phần cứng driver thực tế của dòng Denso RC8 / RC8A.</li>
  <li>Tích hợp camera gắn trên tay máy (Eye-in-Hand) kết hợp mô hình AI nhận diện vật thể để robot tự động định vị phôi và bù trừ sai số gá đặt thời gian thực.</li>
</ul>

<h1 style="margin-top:20pt;">TÀI LIỆU THAM KHẢO (REFERENCES)</h1>
<ol style="font-size:10pt; line-height:1.5;">
  <li><strong>Denso Wave Incorporated (2020).</strong> <em>DENSO Robot User's Manual: VS-6556 / VS-6577 Series (Document No. 410590-0080)</em>. Aichi, Japan.</li>
  <li><strong>John J. Craig (2005).</strong> <em>Introduction to Robotics: Mechanics and Control (3rd Edition)</em>. Pearson Prentice Hall.</li>
  <li><strong>Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, Giuseppe Oriolo (2009).</strong> <em>Robotics: Modelling, Planning and Control</em>. Springer-Verlag London.</li>
  <li><strong>Mark W. Spong, Seth Hutchinson, M. Vidyasagar (2006).</strong> <em>Robot Modeling and Control</em>. John Wiley & Sons.</li>
  <li><strong>Open Robotics (2024).</strong> <em>ROS 2 Documentation: Jazzy Jalisco Release & ros2_control Framework</em>. https://docs.ros.org/en/jazzy/</li>
  <li><strong>Gazebo Community (2024).</strong> <em>Gazebo Harmonic Documentation & gz-physics API Reference</em>. https://gazebosim.org/docs/harmonic/</li>
  <li><strong>David L. Pieper (1968).</strong> <em>The Kinematics of Manipulators Under Computer Control</em>. Ph.D. Thesis, Stanford University Artificial Intelligence Laboratory.</li>
  <li><strong>Tsuneo Yoshikawa (1985).</strong> <em>Manipulability of Robotic Mechanisms</em>. The International Journal of Robotics Research, Vol. 4, No. 2, pp. 3-9.</li>
</ol>

</body>
</html>
"""

html_content = html_template
for i, img in enumerate([img1, img2, img3, img4, img5, img6, img7, img8], start=1):
    html_content = html_content.replace(f"{{img{i}}}", img)

html_path = '/home/dung/denso_ws/Bao_Cao_Do_An_ROS2_Denso_VS6556_HCMUT_6DOF.html'
with open(html_path, 'w', encoding='utf-8') as f:
    f.write(html_content)

print(f"HTML Report generated successfully: {html_path} ({len(html_content)} bytes)")
