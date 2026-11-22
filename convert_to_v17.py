import os
import shutil
import struct

src_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)"
dst_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_MoiNhat_V17"

if os.path.exists(dst_dir):
    shutil.rmtree(dst_dir)

# 1. Copy entire tree from the latest V20 folder
def copy_tree(s, d):
    os.makedirs(d, exist_ok=True)
    for item in os.listdir(s):
        s_item = os.path.join(s, item)
        d_item = os.path.join(d, item)
        if item.endswith(".lock") or item.startswith("~"):
            continue
        if os.path.isdir(s_item):
            copy_tree(s_item, d_item)
        else:
            shutil.copy2(s_item, d_item)

copy_tree(src_dir, dst_dir)
print("1. Cloned BTL_PLC_SCADA_Nhom3 (1) -> BTL_PLC_SCADA_Nhom3_MoiNhat_V17")

# 2. Generate BTL_PLC_SCADA_Nhom3.ap17
ap20_path = os.path.join(dst_dir, "BTL_PLC_SCADA_Nhom3.ap20")
ap17_path = os.path.join(dst_dir, "BTL_PLC_SCADA_Nhom3.ap17")
with open(ap20_path, "r", encoding="utf-8", errors="ignore") as f:
    xml_content = f.read()

xml_v17 = xml_content.replace('ProjectCompatibilityVersion="20.0.0.0"', 'ProjectCompatibilityVersion="17.0.0.0"')
xml_v17 = xml_v17.replace('ProjectCompatibilityVersion="18.0.0.0"', 'ProjectCompatibilityVersion="17.0.0.0"')
xml_v17 = xml_v17.replace("<ProjectVersion>20.0.0.0</ProjectVersion>", "<ProjectVersion>17.0.0.0</ProjectVersion>")
xml_v17 = xml_v17.replace("<ProjectVersion>18.0.0.0</ProjectVersion>", "<ProjectVersion>17.0.0.0</ProjectVersion>")

with open(ap17_path, "w", encoding="utf-8") as f:
    f.write(xml_v17)
print("2. Generated BTL_PLC_SCADA_Nhom3.ap17")

# Also save ap17 in the original source folder BTL_PLC_SCADA_Nhom3 (1)
with open(os.path.join(src_dir, "BTL_PLC_SCADA_Nhom3.ap17"), "w", encoding="utf-8") as f:
    f.write(xml_v17)

# 3. Patch PEData.plf from V20 original backup
v20_plf_bak = os.path.join(src_dir, "System", "PEData_Goc_V20.plf.bak")
with open(v20_plf_bak, "rb") as f:
    plf_data = bytearray(f.read())

orig_len = len(plf_data)
# Offset 24: 0x06A4 (1700 in little-endian)
plf_data[24:28] = struct.pack("<I", 1700)

plf_bytes = bytes(plf_data)
plf_bytes = plf_bytes.replace(b"V20.0.0.0", b"V17.0.0.0")
plf_bytes = plf_bytes.replace(b":20.00.00.00", b":17.00.00.00")
plf_bytes = plf_bytes.replace(b"\t20.0.0.0", b"\t17.0.0.0")
plf_bytes = plf_bytes.replace(b"_20.0.0", b"_17.0.0")

assert len(plf_bytes) == orig_len, f"Length mismatch! {len(plf_bytes)} vs {orig_len}"

dst_plf = os.path.join(dst_dir, "System", "PEData.plf")
with open(dst_plf, "wb") as f:
    f.write(plf_bytes)
print("3. Patched System/PEData.plf to V17 (1700, 3,303,160 bytes)")

# Also write to src_dir
with open(os.path.join(src_dir, "System", "PEData.plf"), "wb") as f:
    f.write(plf_bytes)

# 4. Patch PEData.idx
src_idx = os.path.join(src_dir, "System", "PEData.idx")
with open(src_idx, "rb") as f:
    idx_data = bytearray(f.read())

orig_idx_len = len(idx_data)
# Offset 24: 0x06A4 (1700 in little-endian)
idx_data[24:28] = struct.pack("<I", 1700)
assert len(idx_data) == orig_idx_len

dst_idx = os.path.join(dst_dir, "System", "PEData.idx")
with open(dst_idx, "wb") as f:
    f.write(idx_data)
print("4. Patched System/PEData.idx offset 24 to V17 (1700, 71,515 bytes)")

# Also write to src_dir
with open(os.path.join(src_dir, "System", "PEData.idx"), "wb") as f:
    f.write(idx_data)

# 5. Zip dst_dir
zip_dst = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_MoiNhat_V17"
shutil.make_archive(zip_dst, "zip", dst_dir)
print("5. Created zip:", zip_dst + ".zip")
