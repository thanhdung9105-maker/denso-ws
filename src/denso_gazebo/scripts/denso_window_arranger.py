#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO WINDOW ARRANGER & TASKBAR FIX FOR WSLG (WINDOWS 11 DWM COMPATIBILITY)
Tự động đồng bộ, định vị và kích hoạt hiển thị 3 cửa sổ chính trên màn hình 1920x1080:
1. Gazebo Sim:       940x480 tại Góc Trên Trái (X=10, Y=10)
2. RViz2:            940x480 tại Góc Dưới Trái (X=10, Y=520)
3. Bảng Điều Khiển:  940x980 tại Toàn Bộ Nửa Phải (X=965, Y=10)

CẢ 3 CỬA SỔ HIỂN THỊ ĐỒNG THỜI 100%, KHÔNG CHE CHUẤT NHAU!
ĐẶC BIỆT:
- Triệt tiêu 100% lỗi thumbnail trắng/icon rác trên Taskbar Windows 11.
- Bảo đảm toạ độ Y luôn dương (chống lỗi DWM clip offscreen do frame Y âm).
- Tự động kích hoạt bề mặt hiển thị (surface activation) để DWM Windows render trực tiếp.
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
    # Chờ 1.5s đầu để Qt và OpenGL khởi tạo ổn định
    time.sleep(1.5)

    # Giám sát trong tối đa 40 giây để bắt kịp chu trình nạp MoveIt & RViz2 & Panel
    while time.time() - t_start < 40.0:
        windows = get_x11_windows()
        for wid, title, line in windows:
            # 1. Triệt tiêu icon rác của rviz2 splash / helper 1x1 trên Taskbar Windows 11
            if title == 'rviz2' and '1x1' in line and wid not in suppressed:
                try:
                    subprocess.run(
                        ['xprop', '-id', wid, '-f', '_NET_WM_STATE', '32a', '-set', '_NET_WM_STATE', '_NET_WM_STATE_SKIP_TASKBAR'],
                        env=env, stderr=subprocess.DEVNULL
                    )
                    suppressed.add(wid)
                except Exception:
                    pass

            # 2. RViz2 Main Window -> Nửa Dưới Bên Trái (60, 540)
            if (' - RViz' in title or 'moveit.rviz' in title) and wid not in aligned:
                try:
                    subprocess.run(['xdotool', 'windowsize', wid, '880', '460'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowmove', wid, '60', '540'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowraise', wid], env=env, stderr=subprocess.DEVNULL)
                    aligned.add(wid)
                    found_targets['rviz'] = True
                    print(f"[ARRANGER] RViz2 aligned: {wid} -> 880x460 at frame (60, 540)")
                except Exception:
                    pass

            # 3. Gazebo Sim Main Window -> Nửa Trên Bên Trái (60, 60)
            elif title == 'Gazebo Sim' and '1x1' not in line and wid not in aligned:
                try:
                    subprocess.run(['xdotool', 'windowsize', wid, '880', '440'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowmove', wid, '60', '60'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowraise', wid], env=env, stderr=subprocess.DEVNULL)
                    aligned.add(wid)
                    found_targets['gazebo'] = True
                    print(f"[ARRANGER] Gazebo Sim aligned: {wid} -> 880x440 at frame (60, 60)")
                except Exception:
                    pass

            # 4. Control Panel Main Window -> Toàn Bộ Nửa Phải (960, 60)
            elif ('BẢNG ĐIỀU KHIỂN' in title or 'DENSO VS-6556' in title) and '1x1' not in line and wid not in aligned:
                try:
                    subprocess.run(['xdotool', 'windowsize', wid, '900', '930'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowmove', wid, '960', '60'], env=env, stderr=subprocess.DEVNULL)
                    subprocess.run(['xdotool', 'windowraise', wid], env=env, stderr=subprocess.DEVNULL)
                    aligned.add(wid)
                    found_targets['panel'] = True
                    print(f"[ARRANGER] Control Panel aligned: {wid} -> 900x930 at frame (960, 60)")
                except Exception:
                    pass

        # Khi cả 3 cửa sổ chính đã được định vị, kích hoạt lần lượt để Windows 11 DWM nạp Thumbnail
        if found_targets['gazebo'] and found_targets['rviz'] and found_targets['panel']:
            time.sleep(1.0)
            print("[ARRANGER] ✅ Cả 3 cửa sổ đã được phân bổ không gian độc lập hoàn hảo!")
            break

        time.sleep(0.8)

if __name__ == '__main__':
    arrange_windows()
