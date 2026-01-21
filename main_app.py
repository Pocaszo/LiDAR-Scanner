# LiDAR_VD/apps/main/main_app.py
import threading

from apps.config.config_loader import config
from apps.interface.tcp_socket import TCPSocketInterface
from apps.utils.vd_crc import crc16_lookup
from apps.GUI.scanner_ui import LiDARScannerUI

LISTENER_IP = config.config["client"]["ip"]
LISTENER_PORT = config.config["client"]["port"]
SERVER_IP = config.config["server"]["ip"]
SERVER_PORT = config.config["server"]["port"]
START_MESSAGE_HEX = config.config["client"]["start_message_hex"]

# # ===================== Server =====================

NUM_POINTS = config.config["radar"]["num_points"]
UPDATE_MS = config.config["radar"]["update_ms"]
MAX_DISTANCE_MM = config.config["radar"]["max_distance_mm"]

API_MONITOR = config.config["api"]["monitor"]

ZONES = []
for z in config.config["zones"]:
    ZONES.append((z["min_radius_mm"], z["max_radius_mm"], z["min_angle_deg"], z["max_angle_deg"], z["color"]))

POINT_COUNT = int(config.config["debounce"]["point_count"]) 


lidar_scanner_ui = LiDARScannerUI(NUM_POINTS, MAX_DISTANCE_MM, ZONES)
root = lidar_scanner_ui.root
tcp_interface = TCPSocketInterface(crc16_lookup, root)
tcp_interface.lidar_connect(LISTENER_IP, LISTENER_PORT)
tcp_interface.start_lane_server(SERVER_IP, SERVER_PORT)

def chunk_receiver():
    buffer = b""
    try:
        chunk = tcp_interface.lidar_client_sock.recv(4096)
        if chunk:
            buffer += chunk
            payload, buffer = tcp_interface.process_buffer(buffer)
            if payload is not None:
                distance = tcp_interface.decode_payload(payload)

                lidar_scanner_ui.update_plot(distance, tcp_interface.server_sock, crc16_lookup)
    except BlockingIOError:
        pass  # ไม่มีข้อมูลใหม่
    finally:
        root.after(UPDATE_MS, chunk_receiver)
# -------------------------
# เริ่ม listener thread
# -------------------------
listener_thread = threading.Thread(target=tcp_interface.client_listener, args=(tcp_interface.server_sock,), daemon=True)
listener_thread.start()

# -------------------------
# เริ่มตรวจ queue
# -------------------------
root.after(50, tcp_interface.process_queue)
root.after(UPDATE_MS, chunk_receiver)
root.mainloop()
