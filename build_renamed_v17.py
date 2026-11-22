import os
import shutil
import struct

src_dir = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)"
new_name = "BTL_SCADA_Nhom3_BanMoi_V17"
dst_dir = os.path.join(r"c:\Users\thanh\OneDrive\Máy tính", new_name)

if os.path.exists(dst_dir):
    shutil.rmtree(dst_dir)

# 1. Copy entire directory tree from latest V20 folder
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
print("1. Cloned to", dst_dir)

# 2. Remove old .ap20, .ap18, .ap17 from dst_dir
for f in os.listdir(dst_dir):
    if f.endswith((".ap20", ".ap18", ".ap17", ".info")):
        os.remove(os.path.join(dst_dir, f))

# Read original ap20 from src_dir
ap20_src = os.path.join(src_dir, "BTL_PLC_SCADA_Nhom3.ap20")
with open(ap20_src, "r", encoding="utf-8", errors="ignore") as f:
    xml_content = f.read()

# Replace project name and version in XML
xml_v17 = xml_content.replace('Name="BTL_PLC_SCADA_Nhom3"', f'Name="{new_name}"')
xml_v17 = xml_v17.replace('ProjectCompatibilityVersion="20.0.0.0"', 'ProjectCompatibilityVersion="17.0.0.0"')
xml_v17 = xml_v17.replace('ProjectCompatibilityVersion="18.0.0.0"', 'ProjectCompatibilityVersion="17.0.0.0"')
xml_v17 = xml_v17.replace("<ProjectVersion>20.0.0.0</ProjectVersion>", "<ProjectVersion>17.0.0.0</ProjectVersion>")
xml_v17 = xml_v17.replace("<ProjectVersion>18.0.0.0</ProjectVersion>", "<ProjectVersion>17.0.0.0</ProjectVersion>")

ap17_dst = os.path.join(dst_dir, f"{new_name}.ap17")
with open(ap17_dst, "w", encoding="utf-8") as f:
    f.write(xml_v17)
print("2. Created", ap17_dst)

# 3. Create .info file
info_dst = os.path.join(dst_dir, f"{new_name}.info")
with open(info_dst, "w", encoding="utf-8") as f:
    f.write("""<?xml version="1.0"?>
<InfoFile xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <User>thanh</User>
  <Computer>LAPTOP-489QLSKO</Computer>
  <Time>2026-11-20T00:01:00Z</Time>
</InfoFile>""")
print("3. Created", info_dst)

# 4. Patch PEData.plf from V20 original backup
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

assert len(plf_bytes) == orig_len

dst_plf = os.path.join(dst_dir, "System", "PEData.plf")
with open(dst_plf, "wb") as f:
    f.write(plf_bytes)
print("4. Patched System/PEData.plf (3,303,160 bytes)")

# 5. Patch PEData.idx
src_idx = os.path.join(src_dir, "System", "PEData.idx")
with open(src_idx, "rb") as f:
    idx_data = bytearray(f.read())

orig_idx_len = len(idx_data)
idx_data[24:28] = struct.pack("<I", 1700)
assert len(idx_data) == orig_idx_len

dst_idx = os.path.join(dst_dir, "System", "PEData.idx")
with open(dst_idx, "wb") as f:
    f.write(idx_data)
print("5. Patched System/PEData.idx (71,515 bytes)")

# 6. Make ZIP archive
shutil.make_archive(dst_dir, "zip", dst_dir)
print("6. Created zip:", dst_dir + ".zip")
