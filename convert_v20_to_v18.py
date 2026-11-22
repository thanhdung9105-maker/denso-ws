import os
import shutil

src_v20_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)"
dest_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V18"

print(f"Source V20 folder: {src_v20_dir}")
print(f"Target V18 folder: {dest_dir}")

# 1. Clean and copy the ENTIRE BTL_PLC_SCADA_Nhom3 (1) into BTL_PLC_SCADA_Nhom3_V18
if os.path.exists(dest_dir):
    shutil.rmtree(dest_dir)
shutil.copytree(src_v20_dir, dest_dir)
print("[OK] Copied entire V20 project structure to BTL_PLC_SCADA_Nhom3_V18")

# 2. In target folder, convert BTL_PLC_SCADA_Nhom3.ap20 -> BTL_PLC_SCADA_Nhom3.ap18
ap20_path = os.path.join(dest_dir, "BTL_PLC_SCADA_Nhom3.ap20")
ap18_path = os.path.join(dest_dir, "BTL_PLC_SCADA_Nhom3.ap18")

with open(ap20_path, "r", encoding="utf-8", errors="ignore") as f:
    ap20_text = f.read()

ap18_text = ap20_text.replace('ProjectCompatibilityVersion="20.0.0.0"', 'ProjectCompatibilityVersion="18.0.0.0"')
ap18_text = ap18_text.replace('<ProjectVersion>20.0.0.0</ProjectVersion>', '<ProjectVersion>18.0.0.0</ProjectVersion>')

with open(ap18_path, "w", encoding="utf-8") as f:
    f.write(ap18_text)
print("[OK] Created BTL_PLC_SCADA_Nhom3.ap18 with version 18.0.0.0")

# 3. Patch System/PEData.plf in dest_dir
plf_path = os.path.join(dest_dir, "System", "PEData.plf")
with open(plf_path, "rb") as f:
    data = f.read()

data_v18 = data
data_v18 = data_v18.replace(b'V20.0.0.0', b'V18.0.0.0')
data_v18 = data_v18.replace(b':20.00.00.00', b':18.00.00.00')
data_v18 = data_v18.replace(b'_20.0.0', b'_18.0.0')
data_v18 = data_v18.replace(b'\t20.0.0.0', b'\t18.0.0.0')

data_v18 = bytearray(data_v18)
if data_v18[24:28] == b'\xd0\x07\x00\x00':
    data_v18[24:28] = b'\x08\x07\x00\x00'
    print("[OK] Patched PEData.plf binary header to 1800 (0x0708)")

with open(plf_path, "wb") as f:
    f.write(data_v18)

# 4. Also copy the complete SCL/DB/Tags/SQL sources into dest_dir/Source_Files for safety
src_source_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V17\Source_Import_V17"
if os.path.exists(src_source_dir):
    dest_source_dir = os.path.join(dest_dir, "Source_Blocks_And_Tags")
    if os.path.exists(dest_source_dir):
        shutil.rmtree(dest_source_dir)
    shutil.copytree(src_source_dir, dest_source_dir)
    print("[OK] Added complete Source_Blocks_And_Tags into project")

# 5. ALSO put BTL_PLC_SCADA_Nhom3.ap18 directly inside BTL_PLC_SCADA_Nhom3 (1) folder!
ap18_in_v20_folder = os.path.join(src_v20_dir, "BTL_PLC_SCADA_Nhom3.ap18")
with open(ap18_in_v20_folder, "w", encoding="utf-8") as f:
    f.write(ap18_text)
print(f"[OK] Also placed BTL_PLC_SCADA_Nhom3.ap18 directly into '{src_v20_dir}'")

# Patch plf in src_v20_dir as well or keep backup
plf_in_v20 = os.path.join(src_v20_dir, "System", "PEData.plf")
backup_plf = os.path.join(src_v20_dir, "System", "PEData_Goc_V20.plf.bak")
if not os.path.exists(backup_plf):
    shutil.copy2(plf_in_v20, backup_plf)
    print("[OK] Created backup of original V20 PEData.plf")

with open(plf_in_v20, "wb") as f:
    f.write(data_v18)
print("[OK] Patched PEData.plf inside BTL_PLC_SCADA_Nhom3 (1) as well")

# 6. Create ZIP archive on Desktop
shutil.make_archive(r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V18", 'zip', dest_dir)
shutil.copy2(r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V18.zip", r"c:\Users\thanh\Desktop\BTL_PLC_SCADA_Nhom3_V18.zip")
print("[OK] Updated BTL_PLC_SCADA_Nhom3_V18.zip on Desktop")
