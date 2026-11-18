#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RASPBERRY PI WI-FI G-CODE RECEIVER & SERIAL FORWARDER
------------------------------------------------------
Chạy trên Raspberry Pi (hoặc máy tính nhúng gắn trên robot thật):
1. Kết nối qua Wi-Fi tới máy tính chạy ROS 2 (IP: Port 8888).
2. Nhận dòng lệnh G-code thời gian thực khi người dùng điều khiển góc khớp trên ROS 2.
3. Đẩy trực tiếp lệnh G-code ra cổng nối tiếp Serial (UART / USB: /dev/ttyUSB0 hoặc /dev/ttyACM0)
   để điều khiển bo mạch động cơ bước / driver servo của robot thật.
4. Đọc phản hồi 'ok' từ vi điều khiển và báo cáo trạng thái về ROS 2.

Sử dụng:
    python3 raspberry_pi_gcode_receiver.py --ip 192.168.88.107 --port 8888 --mock
    python3 raspberry_pi_gcode_receiver.py --ip 192.168.88.107 --serial /dev/ttyUSB0
"""

import sys
import os
import time
import socket
import argparse
import datetime

# Kiểm tra thư viện PySerial
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


def parse_arguments():
    parser = argparse.ArgumentParser(description="Raspberry Pi G-Code Wi-Fi Receiver for Denso VS-6556")
    parser.add_argument("--ip", type=str, default="192.168.88.107", help="Địa chỉ IP của máy tính chạy ROS 2")
    parser.add_argument("--port", type=int, default=8888, help="Cổng TCP Socket của ROS 2 Server (mặc định: 8888)")
    parser.add_argument("--serial", type=str, default="/dev/ttyUSB0", help="Cổng Serial nối tới bo mạch Driver robot (/dev/ttyUSB0, /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate cổng Serial (mặc định: 115200)")
    parser.add_argument("--mock", action="store_true", help="Chế độ giả lập (không cần cắm bo mạch thật, in ra màn hình)")
    return parser.parse_args()


def main():
    args = parse_arguments()

    print("=" * 65)
    print("🤖 RASPBERRY PI G-CODE RECEIVER & ROBOT CONTROLLER")
    print("=" * 65)
    print(f"📡 Máy tính ROS 2 Server: {args.ip}:{args.port}")
    print(f"🔌 Cổng Serial Hardware:   {args.serial} (Baudrate: {args.baud})")
    print(f"🧪 Chế độ Giả lập (Mock):  {'BẬT (Mô phỏng)' if args.mock else 'TẮT (Phần cứng thật)'}")
    print("=" * 65)

    ser = None
    if not args.mock:
        if not HAS_SERIAL:
            print("⚠️ CẢNH BÁO: Chưa cài đặt thư viện 'pyserial'. Hãy chạy: pip3 install pyserial")
            print("👉 Tạm thời chuyển sang chế độ MOCK (Giả lập màn hình).")
            args.mock = True
        else:
            try:
                ser = serial.Serial(args.serial, args.baud, timeout=1.0)
                time.sleep(2.0)  # Đợi vi điều khiển khởi động lại sau khi mở cổng
                print(f"✅ Đã kết nối cổng Serial thành công tới Driver robot tại: {args.serial}")
            except Exception as e:
                print(f"❌ Không thể mở cổng Serial {args.serial}: {e}")
                print("👉 Tự động chuyển sang chế độ MOCK để kiểm tra đường truyền Wi-Fi.")
                args.mock = True

    # Vòng lặp kết nối và duy trì qua Wi-Fi
    while True:
        try:
            print(f"\n🔄 Đang kết nối Wi-Fi tới máy chủ ROS 2 tại {args.ip}:{args.port}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((args.ip, args.port))
            sock.settimeout(None)  # Chuyển sang blocking mode sau khi kết nối
            print(f"🎉 KẾT NỐI WI-FI THÀNH CÔNG! Đang lắng nghe góc robot từ ROS 2...\n")

            buffer = ""
            cmd_count = 0

            while True:
                data = sock.recv(2048)
                if not data:
                    print("⚠️ Mất kết nối từ máy chủ ROS 2.")
                    break

                buffer += data.decode('utf-8', errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    cmd_count += 1
                    print(f"[{now_str}] 📥 G-CODE #{cmd_count:04d}: \033[92m{line}\033[0m")

                    # Gửi tới phần cứng thật qua cổng Serial
                    if not args.mock and ser and ser.is_open:
                        try:
                            ser.write((line + "\n").encode('utf-8'))
                            # Đọc phản hồi 'ok' từ vi điều khiển (Marlin / GRBL / STM32)
                            reply = ser.readline().decode('utf-8', errors='ignore').strip()
                            if reply:
                                # Gửi ACK ngược về cho máy chủ ROS 2
                                sock.sendall((reply + "\n").encode('utf-8'))
                        except Exception as err:
                            print(f"❌ Lỗi ghi Serial: {err}")
                    else:
                        # Chế độ Mock: Gửi 'ok' phản hồi ngay lập tức
                        sock.sendall(b"ok\n")

        except KeyboardInterrupt:
            print("\n⏹ Đã dừng chương trình.")
            break
        except Exception as e:
            print(f"❌ Lỗi kết nối: {e}")
            print("⏳ Thử kết nối lại sau 2 giây...")
            time.sleep(2.0)
        finally:
            try:
                sock.close()
            except Exception:
                pass

    if ser and ser.is_open:
        ser.close()
        print("🔌 Đã đóng cổng Serial.")


if __name__ == '__main__':
    main()

