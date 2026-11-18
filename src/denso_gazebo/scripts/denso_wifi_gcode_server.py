#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 WI-FI G-CODE STREAMING SERVER & REST API (ROS 2 <-> RASPBERRY PI)
--------------------------------------------------------------------------------
Node máy chủ Wi-Fi đa luồng trên ROS 2:
1. Lắng nghe góc khớp thời gian thực từ ROS 2 (/joint_states, /arm_controller/joint_trajectory).
2. Tự động chuyển đổi góc khớp sang lệnh G-code chuẩn 6 trục (Joint Deg hoặc Cartesian mm).
3. Truyền G-code thời gian thực qua Wi-Fi TCP Socket (Port 8888) đến Raspberry Pi.
4. Cung cấp HTTP REST API & Web Dashboard (Port 8080) để giám sát và gửi lệnh từ xa.

Tác giả: Nhóm Đề tài Denso VS-6556 HCMUT
Hệ thống: ROS 2 Jazzy | Raspberry Pi Wi-Fi Bridge
"""

import os
import sys
import math
import time
import json
import socket
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from typing import List, Dict, Tuple, Optional

# Thêm đường dẫn tới Denso Dynamics Engine
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "denso_vs6556_moveit_config", "scripts"))
try:
    from denso_dynamics_engine import DensoDynamicsEngine, JOINT_NAMES, JOINT_LIMITS
except ImportError:
    DensoDynamicsEngine = None
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# ROS 2 imports (graceful fallback nếu chạy ở chế độ standalone test)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object


def get_local_wifi_ip() -> str:
    """Lấy địa chỉ IP mạng nội bộ Wi-Fi của máy tính đang chạy server."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


class WiFiGCodeBridgeState:
    """Kho dữ liệu chia sẻ giữa luồng ROS 2, luồng TCP Socket và luồng HTTP Server."""
    def __init__(self):
        self.lock = threading.Lock()
        self.current_joints_rad = [0.0] * 6
        self.current_joints_deg = [0.0] * 6
        self.last_sent_joints_deg = [999.0] * 6
        self.tcp_pos_mm = [0.0, 0.0, 0.0]
        self.tcp_rpy_deg = [0.0, 0.0, 0.0]
        self.last_gcode = "G28 ; Initial Home"
        self.total_gcode_sent = 0
        self.connected_clients: List[str] = []
        self.stream_enabled = True
        self.gcode_mode = "joint"   # 'joint' (A B C U V W), 'cartesian' (X Y Z A B C), 'denso_pac'
        self.feedrate = 1500        # mm/min or deg/min
        self.threshold_deg = 0.15   # Ngưỡng góc khớp thay đổi để gửi G-code mới (tránh nghẽn Wi-Fi)
        self.rate_limit_hz = 25.0   # Tần số phát tối đa qua Wi-Fi
        self.last_send_time = 0.0


SHARED_STATE = WiFiGCodeBridgeState()
DYNAMICS_ENGINE = DensoDynamicsEngine() if DensoDynamicsEngine is not None else None


def format_gcode_command(q_deg: List[float], mode: str = "joint", feedrate: int = 1500) -> Tuple[str, List[float], List[float]]:
    """
    Chuyển đổi 6 góc khớp (độ) sang câu lệnh G-code tương ứng cho robot thật.
    """
    pos_mm = [0.0, 0.0, 0.0]
    rpy_deg = [0.0, 0.0, 0.0]

    # Tính Động học thuận FK để có tọa độ đầu công cụ nếu có engine
    if DYNAMICS_ENGINE is not None:
        try:
            q_rad = [math.radians(a) for a in q_deg]
            fk_res = DYNAMICS_ENGINE.compute_forward_kinematics(q_rad)
            pos_mm = [round(float(v), 2) for v in fk_res.get('position_mm', [0, 0, 0])]
            rpy_deg = [round(float(v), 2) for v in fk_res.get('rpy_deg', [0, 0, 0])]
        except Exception:
            pass

    q_round = [round(a, 2) for a in q_deg]

    if mode == "cartesian":
        # Chuẩn Cartesian G-code: X, Y, Z, A, B, C
        gcode = (
            f"G01 X{pos_mm[0]:.2f} Y{pos_mm[1]:.2f} Z{pos_mm[2]:.2f} "
            f"A{rpy_deg[0]:.2f} B{rpy_deg[1]:.2f} C{rpy_deg[2]:.2f} F{feedrate}"
        )
    elif mode == "denso_pac":
        # Chuẩn Denso PAC / Cao cấp: MOVE P, (X, Y, Z, Rx, Ry, Rz)
        gcode = f"MOVE P, ({pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}, {rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f})"
    else:
        # Chuẩn 6 trục góc quay động cơ (6-Axis Joint Rotation):
        # A = J1, B = J2, C = J3, U = J4, V = J5, W = J6
        gcode = (
            f"G01 A{q_round[0]:.2f} B{q_round[1]:.2f} C{q_round[2]:.2f} "
            f"U{q_round[3]:.2f} V{q_round[4]:.2f} W{q_round[5]:.2f} F{feedrate}"
        )

    return gcode, pos_mm, rpy_deg


# -----------------------------------------------------------------------------
# 1. TCP SOCKET SERVER (KÊNH PHÁT G-CODE STREAMING TỐC ĐỘ CAO CHO RASPBERRY PI)
# -----------------------------------------------------------------------------
class WiFiGCodeTCPServer:
    """Server TCP Socket chịu tải phát dữ liệu G-code tốc độ cao cho Raspberry Pi."""
    def __init__(self, host: str = "0.0.0.0", port: int = 8888):
        self.host = host
        self.port = port
        self.server_sock = None
        self.running = False
        self.client_sockets: List[socket.socket] = []
        self.clients_lock = threading.Lock()

    def start(self):
        self.running = True
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        threading.Thread(target=self._accept_loop, daemon=True, name="WiFi_Accept_Thread").start()
        print(f"📡 [Wi-Fi G-Code TCP Server] Đang lắng nghe kết nối tại port {self.port}...")

    def _accept_loop(self):
        while self.running:
            try:
                client_sock, addr = self.server_sock.accept()
                client_ip_port = f"{addr[0]}:{addr[1]}"
                with self.clients_lock:
                    self.client_sockets.append(client_sock)
                with SHARED_STATE.lock:
                    SHARED_STATE.connected_clients.append(client_ip_port)

                print(f" Raspberry Pi ĐÃ KẾT NỐI Wi-Fi: {client_ip_port}")
                # Gửi câu chào mừng và thiết lập cấu hình ban đầu
                welcome_msg = "; CONNECTED TO DENSO VS-6556 ROS 2 WI-FI BRIDGE\nG21\nG90\n"
                client_sock.sendall(welcome_msg.encode('utf-8'))

                # Tạo luồng đọc phản hồi (ACK / OK) từ Raspberry Pi
                threading.Thread(target=self._client_read_loop, args=(client_sock, client_ip_port), daemon=True).start()
            except Exception as e:
                if not self.running:
                    break
                time.sleep(0.5)

    def _client_read_loop(self, client_sock: socket.socket, client_ip_port: str):
        """Lắng nghe phản hồi từ Raspberry Pi (ví dụ 'ok', 'busy', 'error')."""
        while self.running:
            try:
                data = client_sock.recv(1024)
                if not data:
                    break
                resp = data.decode('utf-8', errors='ignore').strip()
                # print(f"📥 Phản hồi từ Pi ({client_ip_port}): {resp}")
            except Exception:
                break

        # Ngắt kết nối
        print(f"⚠️ Raspberry Pi ({client_ip_port}) đã ngắt kết nối.")
        with self.clients_lock:
            if client_sock in self.client_sockets:
                self.client_sockets.remove(client_sock)
        with SHARED_STATE.lock:
            if client_ip_port in SHARED_STATE.connected_clients:
                SHARED_STATE.connected_clients.remove(client_ip_port)
        try:
            client_sock.close()
        except Exception:
            pass

    def broadcast_gcode(self, gcode_line: str):
        """Phát một dòng G-code tới tất cả các Raspberry Pi đang kết nối qua Wi-Fi."""
        if not gcode_line.endswith('\n'):
            gcode_line += '\n'
        payload = gcode_line.encode('utf-8')

        with self.clients_lock:
            dead_clients = []
            for sock in self.client_sockets:
                try:
                    sock.sendall(payload)
                except Exception:
                    dead_clients.append(sock)

            for dead in dead_clients:
                self.client_sockets.remove(dead)
                try:
                    dead.close()
                except Exception:
                    pass

    def stop(self):
        self.running = False
        if self.server_sock:
            try:
                self.server_sock.close()
            except Exception:
                pass


TCP_SERVER = WiFiGCodeTCPServer(host="0.0.0.0", port=8888)


# -----------------------------------------------------------------------------
# 2. HTTP REST API & WEB DASHBOARD (GIÁM SÁT TRẠNG THÁI VÀ GỬI LỆNH QUA WEB)
# -----------------------------------------------------------------------------
class WiFiGCodeHTTPHandler(BaseHTTPRequestHandler):
    """Xử lý các request HTTP REST API từ web browser hoặc từ script của Raspberry Pi."""

    def log_message(self, format, *args):
        # Tắt log mặc định để tránh tràn terminal
        return

    def _send_json(self, status_code: int, data: dict):
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        self.wfile.write(json.dumps(data, indent=2, ensure_ascii=False).encode('utf-8'))

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/status":
            with SHARED_STATE.lock:
                data = {
                    "status": "ONLINE",
                    "server_ip": get_local_wifi_ip(),
                    "tcp_port": 8888,
                    "stream_enabled": SHARED_STATE.stream_enabled,
                    "gcode_mode": SHARED_STATE.gcode_mode,
                    "feedrate": SHARED_STATE.feedrate,
                    "connected_pi_clients": list(SHARED_STATE.connected_clients),
                    "total_gcode_sent": SHARED_STATE.total_gcode_sent,
                    "last_gcode": SHARED_STATE.last_gcode,
                    "current_joints_deg": [round(a, 2) for a in SHARED_STATE.current_joints_deg],
                    "tcp_position_mm": SHARED_STATE.tcp_pos_mm,
                    "tcp_rpy_deg": SHARED_STATE.tcp_rpy_deg,
                }
            self._send_json(200, data)

        elif path == "/api/gcode/current":
            with SHARED_STATE.lock:
                data = {
                    "gcode": SHARED_STATE.last_gcode,
                    "joints_deg": SHARED_STATE.current_joints_deg,
                    "tcp_mm": SHARED_STATE.tcp_pos_mm
                }
            self._send_json(200, data)

        elif path == "/" or path == "/index.html":
            self._serve_web_dashboard()
        else:
            self._send_json(404, {"error": "Not Found"})

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length).decode('utf-8') if length > 0 else "{}"
        try:
            payload = json.loads(body)
        except Exception:
            payload = {}

        if path == "/api/send_gcode":
            cmd = payload.get("gcode", "").strip()
            if cmd:
                TCP_SERVER.broadcast_gcode(cmd)
                with SHARED_STATE.lock:
                    SHARED_STATE.last_gcode = cmd
                    SHARED_STATE.total_gcode_sent += 1
                self._send_json(200, {"success": True, "sent_command": cmd})
            else:
                self._send_json(400, {"error": "Thiếu trường 'gcode'"})

        elif path == "/api/set_mode":
            mode = payload.get("mode")
            if mode in ["joint", "cartesian", "denso_pac"]:
                with SHARED_STATE.lock:
                    SHARED_STATE.gcode_mode = mode
                self._send_json(200, {"success": True, "current_mode": mode})
            else:
                self._send_json(400, {"error": "Mode phải là 'joint', 'cartesian' hoặc 'denso_pac'"})

        elif path == "/api/toggle_stream":
            enabled = payload.get("enabled")
            with SHARED_STATE.lock:
                if enabled is not None:
                    SHARED_STATE.stream_enabled = bool(enabled)
                else:
                    SHARED_STATE.stream_enabled = not SHARED_STATE.stream_enabled
                curr = SHARED_STATE.stream_enabled
            self._send_json(200, {"success": True, "stream_enabled": curr})

        else:
            self._send_json(404, {"error": "Endpoint không tồn tại"})

    def _serve_web_dashboard(self):
        """Trang Web Dashboard HTML trực quan điều khiển Wi-Fi G-code tích hợp sẵn."""
        wifi_ip = get_local_wifi_ip()
        html = f"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<title>Denso VS-6556 Wi-Fi G-Code API Dashboard</title>
<style>
    body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; }}
    .container {{ max-width: 960px; margin: 0 auto; }}
    .header {{ background: #1e293b; padding: 20px; border-radius: 12px; border-left: 6px solid #10b981; margin-bottom: 20px; }}
    h1 {{ margin: 0 0 8px 0; font-size: 24px; color: #38bdf8; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 16px; margin-bottom: 20px; }}
    .card {{ background: #1e293b; padding: 18px; border-radius: 10px; border: 1px solid #334155; }}
    .card-title {{ font-size: 13px; text-transform: uppercase; color: #94a3b8; margin-bottom: 8px; font-weight: bold; }}
    .val-huge {{ font-size: 20px; font-weight: bold; color: #10b981; font-family: Consolas, monospace; }}
    .gcode-box {{ background: #020617; border: 1px solid #10b981; border-radius: 8px; padding: 15px; font-family: Consolas, monospace; font-size: 16px; color: #4ade80; margin-bottom: 20px; word-break: break-all; }}
    button {{ background: #2563eb; color: white; border: none; padding: 10px 18px; border-radius: 6px; font-weight: bold; cursor: pointer; margin-right: 8px; }}
    button:hover {{ background: #1d4ed8; }}
    .btn-stop {{ background: #dc2626; }}
    .btn-stop:hover {{ background: #b91c1c; }}
    .badge {{ display: inline-block; padding: 4px 10px; border-radius: 20px; font-size: 12px; font-weight: bold; }}
    .badge-green {{ background: #065f46; color: #34d399; }}
    .badge-yellow {{ background: #78350f; color: #fde047; }}
    table {{ width: 100%; border-collapse: collapse; margin-top: 10px; font-size: 14px; }}
    th, td {{ padding: 8px 12px; text-align: left; border-bottom: 1px solid #334155; }}
    th {{ color: #94a3b8; }}
</style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>🤖 DENSO VS-6556 WI-FI G-CODE BRIDGE</h1>
        <div>IP Máy Chủ Wi-Fi: <strong style="color: #38bdf8;">{wifi_ip}</strong> | TCP Port G-code: <strong style="color: #f59e0b;">8888</strong> | HTTP API: <strong style="color: #a855f7;">8080</strong></div>
    </div>

    <div class="card" style="margin-bottom: 20px;">
        <div class="card-title">DÒNG LỆNH G-CODE PHÁT THỜI GIAN THỰC SANG RASPBERRY PI</div>
        <div class="gcode-box" id="last_gcode">Đang kết nối ROS 2...</div>
        <div>
            <button onclick="toggleStream()" id="btn_stream">⏸️ Tạm Dừng Stream</button>
            <button onclick="sendHome()">⌂ G28 (Home Robot)</button>
            <button onclick="setMode('joint')">Khớp (A B C U V W)</button>
            <button onclick="setMode('cartesian')">Cartesian (X Y Z)</button>
        </div>
    </div>

    <div class="grid">
        <div class="card">
            <div class="card-title">KẾT NỐI RASPBERRY PI QUA WI-FI</div>
            <div id="client_status" class="val-huge" style="font-size: 16px;">Đang quét...</div>
            <p style="font-size: 12px; color: #94a3b8; margin-top: 10px;">
                Cú pháp chạy trên Pi:<br>
                <code>python3 raspberry_pi_gcode_receiver.py --ip {wifi_ip}</code>
            </p>
        </div>
        <div class="card">
            <div class="card-title">TỌA ĐỘ ĐẦU CÔNG CỤ TCP (MM)</div>
            <div class="val-huge" id="tcp_val">X: 0.0 | Y: 0.0 | Z: 0.0</div>
            <div style="font-size: 13px; color: #94a3b8; margin-top: 8px;" id="rpy_val">R: 0° | P: 0° | Y: 0°</div>
        </div>
        <div class="card">
            <div class="card-title">TỔNG LỆNH G-CODE ĐÃ TRUYỀN</div>
            <div class="val-huge" id="gcode_count">0 câu lệnh</div>
            <div style="font-size: 13px; color: #94a3b8; margin-top: 8px;">Tần số phát: 25 Hz (Lọc rung &gt; 0.15°)</div>
        </div>
    </div>

    <div class="card">
        <div class="card-title">GÓC KHỚP THỜI GIAN THỰC TỪ ROS 2 (J1 - J6)</div>
        <table>
            <thead><tr><th>Khớp</th><th>J1 (Base)</th><th>J2 (Vai)</th><th>J3 (Khuỷu)</th><th>J4 (Cẳng)</th><th>J5 (Cổ)</th><th>J6 (Mặt bích)</th></tr></thead>
            <tbody><tr id="joint_row" style="font-family: Consolas, monospace; font-size: 16px; color: #38bdf8;"><td>Góc (°)</td><td>0.0</td><td>0.0</td><td>0.0</td><td>0.0</td><td>0.0</td><td>0.0</td></tr></tbody>
        </table>
    </div>
</div>

<script>
function updateStatus() {{
    fetch('/api/status')
    .then(r => r.json())
    .then(d => {{
        document.getElementById('last_gcode').innerText = d.last_gcode;
        document.getElementById('gcode_count').innerText = d.total_gcode_sent.toLocaleString() + ' câu lệnh';
        document.getElementById('tcp_val').innerText = `X: ${{d.tcp_position_mm[0]}} | Y: ${{d.tcp_position_mm[1]}} | Z: ${{d.tcp_position_mm[2]}}`;
        document.getElementById('rpy_val').innerText = `R: ${{d.tcp_rpy_deg[0]}}° | P: ${{d.tcp_rpy_deg[1]}}° | Y: ${{d.tcp_rpy_deg[2]}}°`;
        
        let clientHtml = d.connected_pi_clients.length > 0 
            ? `<span class="badge badge-green">🟢 ĐÃ KẾT NỐI (${{d.connected_pi_clients.length}} PI): ${{d.connected_pi_clients.join(', ')}}</span>`
            : `<span class="badge badge-yellow">🟡 ĐANG CHỜ RASPBERRY PI KẾT NỐI...</span>`;
        document.getElementById('client_status').innerHTML = clientHtml;
        
        let jHtml = '<td>Góc (°)</td>';
        d.current_joints_deg.forEach(a => {{ jHtml += `<td>${{a.toFixed(1)}}°</td>`; }});
        document.getElementById('joint_row').innerHTML = jHtml;

        document.getElementById('btn_stream').innerText = d.stream_enabled ? '⏸️ Tạm Dừng Stream' : '▶️ Bật Lại Stream';
    }});
}}
setInterval(updateStatus, 300);

function toggleStream() {{ fetch('/api/toggle_stream', {{method: 'POST'}}).then(updateStatus); }}
function sendHome() {{ fetch('/api/send_gcode', {{method: 'POST', headers: {{'Content-Type': 'application/json'}}, body: JSON.stringify({{gcode: 'G28'}})}}); }}
function setMode(m) {{ fetch('/api/set_mode', {{method: 'POST', headers: {{'Content-Type': 'application/json'}}, body: JSON.stringify({{mode: m}})}}); }}
</script>
</body>
</html>"""
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(html.encode('utf-8'))


def run_http_server(host: str = "0.0.0.0", port: int = 8080):
    server = HTTPServer((host, port), WiFiGCodeHTTPHandler)
    print(f"🌐 [Web REST API Dashboard] Đã sẵn sàng tại http://{get_local_wifi_ip()}:{port}")
    server.serve_forever()


# -----------------------------------------------------------------------------
# 3. ROS 2 NODE CHÍNH (LẮNG NGHE GÓC KHỚP VÀ KÍCH HOẠT PHÁT G-CODE WI-FI)
# -----------------------------------------------------------------------------
if HAS_ROS2:
    class DensoWiFiGCodeBridgeNode(Node):
        def __init__(self):
            super().__init__('denso_wifi_gcode_bridge')
            self.get_logger().info('Khởi động Denso Wi-Fi G-Code Bridge Node...')

            # 1. Lắng nghe góc khớp thời gian thực từ ROS 2
            self.sub_joint_states = self.create_subscription(
                JointState,
                '/joint_states',
                self.on_joint_states,
                10
            )

            # 2. Lắng nghe quỹ đạo từ MoveIt hoặc Action Trajectory
            self.sub_traj = self.create_subscription(
                JointTrajectory,
                '/arm_controller/joint_trajectory',
                self.on_trajectory,
                10
            )

            # 3. Lắng nghe lệnh chuỗi thủ công
            self.sub_cmd = self.create_subscription(
                String,
                '/denso/gcode_cmd',
                self.on_custom_gcode,
                10
            )

            self.get_logger().info(f'✅ Wi-Fi Server đã sẵn sàng! IP: {get_local_wifi_ip()} | Port: 8888')

        def on_joint_states(self, msg: JointState):
            # Khớp tên khớp và chuyển vị trí sang radian
            q_rad = [0.0] * 6
            matched = 0
            for i, name in enumerate(JOINT_NAMES):
                if name in msg.name:
                    idx = msg.name.index(name)
                    if idx < len(msg.position):
                        q_rad[i] = float(msg.position[idx])
                        matched += 1

            if matched < 6:
                return

            q_deg = [math.degrees(r) for r in q_rad]
            now = time.time()

            with SHARED_STATE.lock:
                SHARED_STATE.current_joints_rad = list(q_rad)
                SHARED_STATE.current_joints_deg = list(q_deg)

                if not SHARED_STATE.stream_enabled:
                    return

                # Bộ lọc tần số (Rate limit)
                if (now - SHARED_STATE.last_send_time) < (1.0 / SHARED_STATE.rate_limit_hz):
                    return

                # Bộ lọc ngưỡng góc (Deadband Filter): Kiểm tra xem có khớp nào đổi > threshold_deg không
                max_diff = max(abs(q_deg[i] - SHARED_STATE.last_sent_joints_deg[i]) for i in range(6))
                if max_diff < SHARED_STATE.threshold_deg:
                    return

                # Tạo câu lệnh G-code
                gcode, pos_mm, rpy_deg = format_gcode_command(
                    q_deg,
                    mode=SHARED_STATE.gcode_mode,
                    feedrate=SHARED_STATE.feedrate
                )

                SHARED_STATE.tcp_pos_mm = pos_mm
                SHARED_STATE.tcp_rpy_deg = rpy_deg
                SHARED_STATE.last_gcode = gcode
                SHARED_STATE.last_sent_joints_deg = list(q_deg)
                SHARED_STATE.last_send_time = now
                SHARED_STATE.total_gcode_sent += 1

            # Phát qua Wi-Fi Socket cho tất cả Raspberry Pi
            TCP_SERVER.broadcast_gcode(gcode)

        def on_trajectory(self, msg: JointTrajectory):
            """Xử lý nạp đón đầu danh sách điểm quỹ đạo liên tục."""
            if not msg.points:
                return
            self.get_logger().info(f" Nhận quỹ đạo gồm {len(msg.points)} điểm. Chuẩn bị stream sang Pi...")

        def on_custom_gcode(self, msg: String):
            cmd = msg.data.strip()
            if cmd:
                TCP_SERVER.broadcast_gcode(cmd)
                with SHARED_STATE.lock:
                    SHARED_STATE.last_gcode = cmd
                    SHARED_STATE.total_gcode_sent += 1


def main():
    print("=" * 70)
    print("🚀 KHỞI ĐỘNG DENSO VS-6556 WI-FI G-CODE BRIDGE SERVER")
    print("=" * 70)
    wifi_ip = get_local_wifi_ip()
    print(f" Địa chỉ IP Wi-Fi Server: {wifi_ip}")
    print(f" Cổng TCP G-Code Stream:  8888  (Kết nối từ Raspberry Pi)")
    print(f" Cổng Web REST API:       8080  (Mở trên trình duyệt: http://{wifi_ip}:8080)")
    print("=" * 70)

    # 1. Khởi chạy TCP Socket Server trong luồng riêng
    TCP_SERVER.start()

    # 2. Khởi chạy HTTP Server trong luồng riêng
    threading.Thread(target=run_http_server, args=("0.0.0.0", 8080), daemon=True, name="HTTP_Thread").start()

    # 3. Khởi chạy ROS 2 Node hoặc vòng lặp giả lập
    if HAS_ROS2:
        rclpy.init()
        node = DensoWiFiGCodeBridgeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\n⏹ Đang dừng server...")
        finally:
            node.destroy_node()
            rclpy.shutdown()
            TCP_SERVER.stop()
    else:
        print("⚠️ ROS 2 chưa nạp trong môi trường hiện tại. Chạy chế độ Standalone Socket & Web Server...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            TCP_SERVER.stop()


if __name__ == '__main__':
    main()

