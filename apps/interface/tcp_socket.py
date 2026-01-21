# tcp_socket.py
import socket
import numpy as np
import queue
import json
from datetime import datetime
from apps.config.config_loader import config

def get_sdatetime():
    now = datetime.now()
    # รูปแบบ: YYYYMMDDHHMMSSmmm
    return now.strftime("%Y%m%d%H%M%S") + f"{int(now.microsecond/1000):03d}"

class TCPSocketInterface:
    def __init__(self, crc16_lookup, root):
        self.lidar_client_sock = None
        self.recv_queue = queue.Queue()  # queue สำหรับส่งข้อความจาก thread → main GUI
        self.crc16_lookup = crc16_lookup
        self.server_sock = None
        self.root = root

    def lidar_connect(self, server_ip: str, server_port: int):
        # =================== Socket Setup ===================
        self.lidar_client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lidar_client_sock.connect((server_ip, server_port))
        self.lidar_client_sock.setblocking(False)
        self.lidar_client_sock.sendall(bytes.fromhex(config.config["client"]["start_message_hex"]))

        print(f"Connected to Lidar {server_ip}:{server_port} and sent start message.")

    def start_lane_server(self, server_ip: str, server_port: int):
        lane_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lane_sock.bind((server_ip, server_port))
        lane_sock.listen(1)
        print(f"Server listening on {server_ip}:{server_port} ...")
        self.server_sock, addr = lane_sock.accept()
        print("Client connected:", addr)

    # -------------------------
    # Thread สำหรับรับข้อมูล
    # -------------------------
    def client_listener(self, sock: socket.socket):
        while True:
            try:
                data = sock.recv(4096)
                if not data:
                    break  # client ปิด connection
                self.recv_queue.put(data)  # ส่งข้อมูลไป GUI thread
            except Exception as e:
                print("[THREAD] Receive error:", e)
                break

    # -------------------------
    # GUI update function
    # -------------------------
    def process_queue(self):
        try:
            while True:
                data = self.recv_queue.get_nowait()
                self.handle_message(data)
        except queue.Empty:
            pass
        finally:
            self.root.after(50, self.process_queue)

    # -------------------------
    # ฟังก์ชัน handle message
    # -------------------------
    def handle_message(self,data: bytes):
        try:
            print("[SERVER] Received raw:", data)
            # -------------------------------
            # ตัด header/footer
            # -------------------------------
            if data.startswith(b"\x02") and data.endswith(b"\x03"):
                core = data[1:-1]
            else:
                core = data

            # -------------------------------
            # แยก JSON + CRC
            # -------------------------------
            if len(core) > 4:
                json_part = core[:-4]
                crc_part = core[-4:]
            else:
                json_part = core
                crc_part = b""

            print(f"[SERVER] ✅ json_part: {json_part}")
            message = json.loads(json_part.decode("utf-8"))
            print(f"[SERVER] message:\n{json.dumps(message, indent=2)}")

            recv_crc = int(crc_part.decode("utf-8"), 16)
            calc_crc = self.crc16_lookup(json_part)
            print(f"[SERVER] CRC calc={calc_crc:04X}, CRC recv={recv_crc:04X}")

            if recv_crc == calc_crc:
                print(f"[SERVER] ✅ CRC OK ({recv_crc:04X})")

                # -------------------------------
                # ส่ง response
                # -------------------------------
                message_status = {
                    "header": {
                        "sender": 116,
                        "receiver": 100,
                        "datetime": get_sdatetime(),
                        "msg": 5,
                        "seq": 10
                    },
                    "data": {"status": 1}
                }
                message_bytes = json.dumps(message_status, separators=(",", ":")).encode("utf-8")
                crc_val = self.crc16_lookup(message_bytes)
                crc_str = f"{crc_val:04X}".encode("utf-8")
                packet = b'\x02' + message_bytes + crc_str + b'\x03'
                self.server_sock.sendall(packet)
                print("[SERVER] Sent response packet:", packet)

                self.lidar_client_sock.sendall(bytes.fromhex(config.config["client"]["start_message_hex"]))
                print("[SERVER] Sent start message to Lidar.")
            else:
                print(f"[SERVER] ❌ CRC FAIL recv={recv_crc:04X} calc={calc_crc:04X}")

        except Exception as e:
            print("[SERVER] handle_message error:", e)

    
# =================== Helper Functions ===================
    def process_buffer(self, buffer: bytes):
        start_idx = buffer.rfind(b"\x02\x02\x02\x02")
        if start_idx == -1 or len(buffer) < start_idx + 85 + 720:
            return None, buffer
        payload = buffer[start_idx + 85:start_idx + 85 + 720]
        buffer = buffer[start_idx + 85 + 720:]
        return payload, buffer

    def decode_payload(self, payload: bytes):
        distance = np.frombuffer(payload, dtype='>u2')
        if len(distance) < config.config["radar"]["num_points"]:
            distance = np.pad(distance, (0, config.config["radar"]["num_points"] - len(distance)), 'constant')
        else:
            distance = distance[:config.config["radar"]["num_points"]]

        distance = np.clip(distance, 0, config.config["radar"]["max_distance_mm"])
        return distance