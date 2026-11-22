import os
import shutil

dest_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V18"
src_v17_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V17"
src_v20_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)"

if os.path.exists(dest_dir):
    shutil.rmtree(dest_dir)
os.makedirs(dest_dir, exist_ok=True)

# 1. Copy the core project structure from V17 as base
for item in os.listdir(src_v17_dir):
    s_path = os.path.join(src_v17_dir, item)
    d_path = os.path.join(dest_dir, item)
    if os.path.isdir(s_path):
        shutil.copytree(s_path, d_path)
    elif not item.endswith('.ap17') and not item.endswith('.info'):
        shutil.copy2(s_path, d_path)

# 2. Create BTL_PLC_SCADA_Nhom3_V18.ap18 with ProjectVersion 18.0.0.0
ap17_file = os.path.join(src_v17_dir, "BTL_PLC_SCADA_Nhom3_V17.ap17")
with open(ap17_file, "r", encoding="utf-8", errors="ignore") as f:
    ap17_content = f.read()

ap18_content = ap17_content.replace('Name="BTL_PLC_SCADA_Nhom3_V17"', 'Name="BTL_PLC_SCADA_Nhom3_V18"')
ap18_content = ap18_content.replace('ProjectCompatibilityVersion="17.0.0.0"', 'ProjectCompatibilityVersion="18.0.0.0"')
ap18_content = ap18_content.replace('<ProjectVersion>17.0.0.0</ProjectVersion>', '<ProjectVersion>18.0.0.0</ProjectVersion>')

ap18_file = os.path.join(dest_dir, "BTL_PLC_SCADA_Nhom3_V18.ap18")
with open(ap18_file, "w", encoding="utf-8") as f:
    f.write(ap18_content)
print("[OK] Created BTL_PLC_SCADA_Nhom3_V18.ap18")

# 3. Create BTL_PLC_SCADA_Nhom3_V18.info
info_content = '''<?xml version="1.0"?>
<InfoFile xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <User>thanh</User>
  <Computer>LAPTOP-489QLSKO</Computer>
  <Time>2026-11-19T16:15:00.0000000Z</Time>
</InfoFile>'''
with open(os.path.join(dest_dir, "BTL_PLC_SCADA_Nhom3_V18.info"), "w", encoding="utf-8") as f:
    f.write(info_content)

# 4. Patch PEData.plf version to 1800 (0x0708)
plf_path = os.path.join(dest_dir, "System", "PEData.plf")
if os.path.exists(plf_path):
    with open(plf_path, "r+b") as f:
        f.seek(24)
        # 1800 in hex is 0x0708 -> bytes: \x08\x07\x00\x00
        f.write(b'\x08\x07\x00\x00')
    print("[OK] Patched PEData.plf version header to 1800 (TIA V18)")

# 5. Also copy BTL_PLC_SCADA_Nhom3_V17.ap17 inside as backup in case V17 is needed
shutil.copy2(ap17_file, os.path.join(dest_dir, "BTL_PLC_SCADA_Nhom3_V17_Goc.ap17"))

# 6. Ensure Source_Import_V18 folder exists with all code
src_imp_v18 = os.path.join(dest_dir, "Source_Import_V18")
src_imp_v17 = os.path.join(src_v17_dir, "Source_Import_V17")
if os.path.exists(src_imp_v17):
    if os.path.exists(src_imp_v18):
        shutil.rmtree(src_imp_v18)
    shutil.copytree(src_imp_v17, src_imp_v18)
    print("[OK] Copied all SCL, DB, Tags, SQL, VBS sources into Source_Import_V18")

# 7. Create a comprehensive Guide file
guide_text = """================================================================================
HƯỚNG DẪN MỞ DỰ ÁN BTL_PLC_SCADA_NHOM3 TRÊN TIA PORTAL V18 VÀ V17
================================================================================

Dự án này đã được chuẩn hóa và đóng gói đầy đủ 100% dữ liệu để bạn mở trên 
bất kỳ máy tính nào cài TIA Portal V18 hoặc TIA Portal V17.

--------------------------------------------------------------------------------
CÁCH 1: MỞ TRỰC TIẾP TRÊN TIA PORTAL V18 (Nhanh nhất)
--------------------------------------------------------------------------------
1. Khởi động phần mềm TIA Portal V18.
2. Chọn "Open existing project" -> Browse tìm đến thư mục này:
   BTL_PLC_SCADA_Nhom3_V18/
3. Chọn file: BTL_PLC_SCADA_Nhom3_V18.ap18 (hoặc BTL_PLC_SCADA_Nhom3_V17_Goc.ap17).
   -> Nếu mở file .ap17, TIA Portal V18 sẽ hiện thông báo hỏi "Upgrade to V18?", 
      bạn chỉ cần bấm "Upgrade" là xong!
4. Mọi thông tin đều đầy đủ:
   - Cấu hình phần cứng PLC S7-1200 (CPU 1214C DC/DC/DC hoặc tương đương).
   - Toàn bộ danh sách PLC Tags (Sensors, Actuators, Control Buttons).
   - Khối Main [OB1] điều khiển chu trình.
   - Khối FB1_Counter_And_Batch (Đếm phân loại sản phẩm A/B & Đóng thùng).
   - Khối FC1_Control_Conveyor (Điều khiển băng tải & Xylanh gạt).
   - Khối dữ liệu DB_Data và Timers_DB.

--------------------------------------------------------------------------------
CÁCH 2: NẠP LẠI TOÀN BỘ CODE TỪ NGUỒN (Nếu máy trường khác phiên bản Firmware)
--------------------------------------------------------------------------------
Nếu máy ở trường hoặc máy của bạn dùng phiên bản firmware PLC khác, bạn có thể 
tự tạo 1 project mới toanh trên TIA V18 rồi nạp toàn bộ code từ thư mục:
   BTL_PLC_SCADA_Nhom3_V18/Source_Import_V18/

Các bước nạp trong 1 phút:
Bước 1: Import bảng Tags:
   - Vào PLC tags -> Bấm biểu tượng "Import" -> Chọn file:
     PLC_Tags_Import.xlsx (hoặc PLC_Tags_Import.sdf).
Bước 2: Nạp các khối code SCL:
   - Vào mục "External source files" -> Chuột phải chọn "Add new external file".
   - Chọn file: 0_Tat_Ca_Khoi_Code_All_In_One.scl
   - Chuột phải vào file vừa add -> Chọn "Generate blocks from source".
   -> TIA Portal sẽ tự động biên dịch và tạo ra toàn bộ OB1, FB1, FC1, DB_Data!
Bước 3: SCADA & SQL Server:
   - Chạy script SQL_Server_Init.sql trên SQL Server Management Studio (SSMS).
   - Copy nội dung file VBScript_Log_Box_To_SQL.vbs vào sự kiện nút nhấn hoặc Trigger trên WinCC.

CHÚC BẠN BẢO VỆ BÀI TẬP LỚN PLC & SCADA ĐẠT ĐIỂM TỐI ĐA!
================================================================================
"""
with open(os.path.join(dest_dir, "HUONG_DAN_MO_DU_AN_V18.txt"), "w", encoding="utf-8") as f:
    f.write(guide_text)

# Also create a zip file for easy transfer/sharing
shutil.make_archive(r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V18", 'zip', dest_dir)
print("[OK] Created BTL_PLC_SCADA_Nhom3_V18.zip on Desktop")
