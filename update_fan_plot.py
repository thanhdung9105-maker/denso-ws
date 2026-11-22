import os

paths = [
    r"c:\Users\thanh\OneDrive\Máy tính\sketch_oct21a\sketch_oct21a.ino",
    r"c:\Users\thanh\Documents\Arduino\sketch_oct21a\sketch_oct21a.ino",
    r"c:\Users\thanh\denso_ws\sketch_oct21a.ino"
]

old_pattern = "float fanPlot = (currentFanPWM > 0) ? (float)currentFanPWM : (1.5 + sin(currentMillis * 0.003) * 1.2);"
new_pattern = "float fanPlot = (float)currentFanPWM; // Dung yen hoan toan tai 0 khi quat khong chay"

for p in paths:
    if os.path.exists(p):
        with open(p, "r", encoding="utf-8") as f:
            content = f.read()
        if old_pattern in content:
            content = content.replace(old_pattern, new_pattern)
            with open(p, "w", encoding="utf-8") as f:
                f.write(content)
            print(f"[OK] Updated {p}")
        else:
            print(f"[WARN] Pattern not found in {p}")
    else:
        print(f"[ERROR] Path not found: {p}")

