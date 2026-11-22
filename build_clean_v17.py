import os
import shutil
import time

src_v17 = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V17"
new_name = "BTL_SCADA_Nhom3_Moi_V17"
dst_dir = os.path.join(r"c:\Users\thanh\OneDrive\Máy tính", new_name)

if os.path.exists(dst_dir):
    shutil.rmtree(dst_dir)

# 1. Copy original genuine V17 project
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

copy_tree(src_v17, dst_dir)
print("1. Copied clean V17 project tree")

# 2. Rename old ap17 and info
for f in os.listdir(dst_dir):
    if f.endswith((".ap17", ".info")):
        os.remove(os.path.join(dst_dir, f))

# 3. Read original ap17 XML and update project name
orig_ap17 = os.path.join(src_v17, "BTL_PLC_SCADA_Nhom3_V17.ap17")
with open(orig_ap17, "r", encoding="utf-8", errors="ignore") as f:
    xml_content = f.read()

xml_new = xml_content.replace('Name="BTL_PLC_SCADA_Nhom3_V17"', f'Name="{new_name}"')

new_ap17_path = os.path.join(dst_dir, f"{new_name}.ap17")
with open(new_ap17_path, "w", encoding="utf-8") as f:
    f.write(xml_new)
print("2. Created", new_ap17_path)

# 4. Create new .info with current time
new_info_path = os.path.join(dst_dir, f"{new_name}.info")
cur_time = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
with open(new_info_path, "w", encoding="utf-8") as f:
    f.write(f"""<?xml version="1.0"?>
<InfoFile xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <User>thanh</User>
  <Computer>LAPTOP-489QLSKO</Computer>
  <Time>{cur_time}</Time>
</InfoFile>""")
print("3. Created", new_info_path)

# 5. Add Export_Full_Thong_Tin_Nhom3 folder with all SCL blocks, DBs, Excel Tags, SQL
export_dir = os.path.join(dst_dir, "Export_Full_Thong_Tin_Nhom3")
src_blocks = r"c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)\Source_Blocks_And_Tags"
if os.path.exists(export_dir):
    shutil.rmtree(export_dir)
shutil.copytree(src_blocks, export_dir)
print("4. Added Export_Full_Thong_Tin_Nhom3 folder")

# 6. Touch all files in dst_dir to current timestamp (Nov 20, 2026)
now = time.time()
for root, dirs, files in os.walk(dst_dir):
    for f in files:
        fp = os.path.join(root, f)
        try:
            os.utime(fp, (now, now))
        except:
            pass

# 7. Zip dst_dir
shutil.make_archive(dst_dir, "zip", dst_dir)
print("5. Created zip:", dst_dir + ".zip")
