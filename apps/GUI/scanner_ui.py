# scanner_ui.py
from datetime import datetime
import socket
import time
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import json
from apps.config.config_loader import config

def get_sdatetime():
    now = datetime.now()
    # รูปแบบ: YYYYMMDDHHMMSSmmm
    return now.strftime("%Y%m%d%H%M%S") + f"{int(now.microsecond/1000):03d}"

def has_consecutive_true_np(mask, n=5):
    idx = np.where(mask)[0]
    if len(idx) < n:
        return False
    return np.any(np.diff(idx) == 1).all() or np.any(
        np.convolve(mask.astype(int), np.ones(n, dtype=int), mode='valid') >= n
    )

class LiDARScannerUI:
    def __init__(self, num_points, max_distance_mm, zones):
        self.root = None
        self.num_points = num_points
        self.max_distance_mm = max_distance_mm
        self.zones = zones
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0

        # ---- State variables (global) ----
        self.zone_states = ["leave"] * len(self.zones)
        self.presence_count = [0] * len(self.zones)
        self.miss_count = [0] * len(self.zones)

        # =================== GUI Setup ===================
        self.root = tk.Tk()
        self.root.title("Military Radar - 180° Sweep Left Zero")
        self.root.configure(bg="#003300")

        fig = plt.Figure(figsize=(8, 8), dpi=100)
        ax = fig.add_subplot(111, polar=True)

        self.theta = np.linspace(0, np.pi, self.num_points)
        distance = np.zeros(self.num_points)
        colors = np.full(self.num_points, 'lime')
        self.sc = ax.scatter(self.theta, distance, c=colors, s=10)

        ax.set_facecolor('#003300')
        ax.set_theta_zero_location('W')  # 0° อยู่ด้านซ้าย
        ax.set_theta_direction(-1)        # sweep ลงขวา
        ax.grid(color='lime', linestyle='--', linewidth=0.5)

        # แสดงองศา 0°-180° บน polar plot
        ax.set_xticks(np.linspace(0, np.pi, 7))
        ax.set_xticklabels([f"{int(np.degrees(x))}°" for x in np.linspace(0, np.pi, 7)], color='lime', fontsize=8)
        ax.set_yticklabels([])  # ไม่ต้องแสดงระยะ

        ax.set_ylim(0, max_distance_mm + 1000)

        self.title_text = ax.set_title("Military Radar - 180° Sweep Left Zero", va='bottom', color='lime')

        # zone patches
        zone_patches = []
        for min_r, max_r, min_a_deg, max_a_deg, color in zones:
            wedge = Wedge(center=(0,0), r=max_r, theta1=min_a_deg, theta2=max_a_deg,
                        facecolor=color, alpha=0.5)
            zone_patches.append((wedge, min_r, max_r))
            ax.add_patch(wedge)
            
        cbar = fig.colorbar(self.sc, ax=ax, orientation='vertical', pad=0.05)
        cbar.set_label('Distance (mm)', color='lime')
        cbar.ax.yaxis.set_tick_params(color='lime')
        plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='lime')

        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # =================== Update Function ===================
    def update_plot(self, distance, server_sock: socket.socket, crc16_lookup):
        # update scatter
        self.sc.set_offsets(np.c_[self.theta, distance])
        # # highlight points ใน zone
        colors = np.full(self.num_points, 'lime')

        point_count = 0
        for idx, (min_r, max_r, min_a_deg, max_a_deg, color) in enumerate(self.zones, start=1):
            min_a = np.radians(min_a_deg)
            max_a = np.radians(max_a_deg)                    
            # mask มุม
            theta_mask = (self.theta >= min_a) & (self.theta <= max_a)                    
            # mask ระยะจาก 0 → point
            r_mask = (distance >= min_r) & (distance <= max_r)

            mask = theta_mask & r_mask
            colors[mask] = color
            prev_state = self.zone_states[idx-1]  # เก็บ state เดิม

            if has_consecutive_true_np(mask, n=5):
                self.presence_count[idx-1] += 1
                self.miss_count[idx-1] = 0
            else:
                self.miss_count[idx-1] += 1
                self.presence_count[idx-1] = 0

            # Update state
            point_count += 1
            if self.presence_count[idx-1] >= int(config.config["debounce"]["frame_on"]):
                self.zone_states[idx-1] = "presence"
                vd_detect = 1       # presence
                vd_direction = 1    # forword
                lidar1_detect = 1   # Lidar detector On
            elif self.miss_count[idx-1] >= int(config.config["debounce"]["frame_off"]):
                self.zone_states[idx-1] = "leave"
                vd_detect = 2 # leave
                vd_direction = 1  # forword
                lidar1_detect = 0 # Lidar detector Off

            # ✅ Print toggle เฉพาะเวลามีการเปลี่ยน state
            if self.zone_states[idx-1] != prev_state:
                print(f"Zone {idx} -> {self.zone_states[idx-1]}")
                # สมมุติข้อมูล JSON ที่จะส่ง
                # -----------------------------
                # สร้าง message แยกเป็น dict
                # -----------------------------                        
                message_dict = {
                    "header": {
                        "sender": 116,
                        "receiver": 100,
                        "datetime": get_sdatetime(),  # <<< เวลาแบบ real-time,
                        "msg": 7,
                        "seq": 10
                    },
                    "data": {
                        "status": 1,
                        "direction": [vd_detect, vd_direction, 0, f"00000000000{lidar1_detect}0000", "0000000000000000"]
                    }
                }
                
                # dict → JSON → bytes
                message_bytes = json.dumps(message_dict, separators=(",", ":")).encode("utf-8")

                # CRC ของ response (lookup table)
                crc_val = crc16_lookup(message_bytes)
                crc_str = f"{crc_val:04X}".encode("utf-8")  # 4-char hex string, bytes

                # build packet
                packet = b'\x02' + message_bytes + crc_str + b'\x03'

                print("[SERVER] Sending packet:", packet)                     
                # ส่ง packet
                server_sock.sendall(packet)
                print("Packet sent to client.")

            # แสดงสี
            if self.zone_states[idx-1] == "presence":
                colors[mask] = color
        self.sc.set_color(colors)

        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_time)
            self.frame_count = 0
            self.last_time = current_time

        max_d = distance.max()
        min_d = distance.min()
        self.title_text.set_text(f"LIDAR Left Zero : Point:{point_count} | FPS: {self.fps:.1f} | Max: {max_d} mm | Min: {min_d} mm")
        
        if (config.config["api"]["monitor"] =='Enable'):
            self.canvas.draw_idle()
        point_count = 0

