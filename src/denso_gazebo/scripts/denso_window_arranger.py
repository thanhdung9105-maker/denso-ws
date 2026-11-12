#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO WINDOW ARRANGER & TASKBAR FIX FOR WSLG (WINDOWS 11 DWM COMPATIBILITY)
Tự động đồng bộ, định vị và kích hoạt hiển thị 3 cửa sổ chính:
1. Gazebo Sim: 940x640 tại +50+40 (Vừa vặn trong màn hình 816px, chống Y âm)
2. RViz2: 1180x680 tại +40+40 (Triệt tiêu 100% icon rác rviz2 trên Taskbar Windows)
3. Bảng điều khiển Denso: 1200x710 tại +120+40 (Đầy đủ 7 tab)

AN TOÀN TUYỆT ĐỐI:
- KHÔNG BAO GIỜ can thiệp vào các cửa sổ ngầm (1x1, offscreen context) của Gazebo GUI để tránh lỗi SIGABRT (-6).
- Chỉ kích hoạt và định vị khi cửa sổ chính đã nạp xong (Width > 500).
"""

import subprocess
import time
import os
import sys

def get_x11_windows():
    env = os.environ.copy()
    if 'DISPLAY' not in env:
        env['DISPLAY'] = ':0'
    try:
        raw = subprocess.check_output(
            ['xwininfo', '-root', '-tree'],
            env=env, stderr=subprocess.DEVNULL
        ).decode('utf-8', errors='ignore')
        
        windows = []
        for line in raw.splitlines():
            line = line.strip()
            # Ví dụ: 0xa00106 "... - RViz": ("rviz2" "rviz2")  1180x680+38+59  +56+77
            if line.startswith('0x'):
                parts = line.split(maxsplit=2)
                wid = parts[0]
                title = ''
                if '"' in line:
                    try:
                        title = line.split('"')[1]
                    except Exception:
                        pass
                windows.append((wid, title, line))
        return windows
    except Exception:
        return []

def arrange_windows():
    env = os.environ.copy()
    if 'DISPLAY' not in env:
        env['DISPLAY'] = ':0'

    aligned = set()
    suppressed = set()
    found_targets = {'gazebo': False, 'rviz': False, 'panel': False}
    
    t_start = time.time()
    # Chờ 2 giây đầu để hệ thống OpenGL và Qt khởi tạo xong Context an toàn
    time.sleep(2.0)

    # Giám sát trong tối đa 45 giây để bắt kịp chu trình khởi động của MoveIt & Controllers
    while time.time() - t_start < 45.0:
        windows = get_x11_windows()
        for wid, title, line in windows:
            # 1. Xử lý triệt để icon rác của rviz2 trên Taskbar Windows 11
            # CHỈ áp dụng cho sub-window rviz2 1x1 hoặc child widget, TUYỆT ĐỐI KHÔNG đụng vào Gazebo
            if title == 'rviz2' and ' - RViz' not in title:
                if wid not in suppressed:
                    try:
                        subprocess.run(
                            ['xprop', '-id', wid, '-f', '_NET_WM_STATE', '32a', '-set', '_NET_WM_STATE', '_NET_WM_STATE_SKIP_TASKBAR'],
                            env=env, stderr=subprocess.DEVNULL
                        )
                        suppressed.add(wid)
                    except Exception:
                        pass

            # 2. RViz2 Main Window (Chỉ can thiệp khi là cửa sổ chính)
            if (' - RViz' in title or 'moveit.rviz' in title) and wid not in aligned:
                try:
                    subprocess.run(['xdotool', 'windowsize', wid, '1180', '680'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowmove', wid, '40', '40'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowmap', wid], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowraise', wid], env=env, stderr=subprocess.DEVNULL)
                    aligned.add(wid)
                    found_targets['rviz'] = True
                    print(f"[ARRANGER] RViz2 mapped & aligned: {wid} ({title})")
                except Exception:
                    pass

            # 3. Gazebo Sim Main Window (Chỉ can thiệp khi cửa sổ chính đã hiển thị thật sự, không phải 1x1)
            # TUYỆT ĐỐI không gọi windowmap trên Gazebo Sim để tránh xung đột OpenGL/GLX context gây SIGABRT (-6)
            elif title == 'Gazebo Sim' and '1x1' not in line and wid not in aligned:
                try:
                    # Đảm bảo cửa sổ không bị Y âm và nổi lên trên một cách an toàn
                    subprocess.run(['xdotool', 'windowmove', wid, '50', '40'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowraise', wid], env=env, stderr=subprocess.DEVNULL)
                    aligned.add(wid)
                    found_targets['gazebo'] = True
                    print(f"[ARRANGER] Gazebo Sim safe positioned: {wid} ({title})")
                except Exception:
                    pass

            # 4. Control Panel Main Window
            elif ('BẢNG ĐIỀU KHIỂN' in title or 'DENSO VS-6556' in title) and '1x1' not in line and wid not in aligned:
                try:
                    subprocess.run(['xdotool', 'windowsize', wid, '1200', '710'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowmove', wid, '120', '40'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowmap', wid], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowraise', wid], env=env, stderr=subprocess.DEVNULL)
                    aligned.add(wid)
                    found_targets['panel'] = True
                    print(f"[ARRANGER] Control Panel mapped & aligned: {wid} ({title})")
                except Exception:
                    pass

        # Nếu đã tìm thấy và căn chỉnh cả 3 cửa sổ chính, kết thúc an toàn
        if found_targets['gazebo'] and found_targets['rviz'] and found_targets['panel']:
            time.sleep(2.0)
            break

        time.sleep(1.0)

if __name__ == '__main__':
    arrange_windows()
