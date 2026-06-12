import cv2
import numpy as np
import customtkinter as ctk
from pythonosc import udp_client
import threading
import time
import math
import json
import os
import csv
from datetime import datetime
from collections import deque
from tkinter import messagebox

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


def rotation_matrix_to_quaternion(R):
    """Конвертация матрицы вращения 3х3 в кватернион [x, y, z, w]"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return qx, qy, qz, qw


class ArUcoTrackerApp:
    def __init__(self):
        self.root = ctk.CTk()
        self.root.title("SlimeVR ArUco Optical Correction")
        self.root.geometry("960x820") 
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.WIN_NAME = "ArUco Preview"
        self.CALIB_WIN_NAME = "Camera Calibration (Press C to capture, ENTER to finish)"
        self.config_file = "config.txt"

        self.cap = None
        self.running = False
        self.preview_running = False
        self.calib_running = False
        self.thread = None
        self.osc_client = None
        
        self.is_logging = False
        self.log_file = None
        self.csv_writer = None
        
        self.marker_buffers = {}
        self.last_ui_log_time = {}

        # Дефолтная матрица (будет перезаписана, если есть калибровка в конфиге)
        self.camera_matrix = np.array([[500.0, 0.0, 320.0],
                                       [0.0, 500.0, 240.0],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.load_config()
        self.create_widgets()

    def load_config(self):
        self.config_data = {
            "cam_index": 0,
            "marker_size": 57.0,
            "osc_ip": "127.0.0.1",
            "osc_port": 9005,
            "marker_ids": "0,1,2,3,4,5,6,7,8,9",
            "min_angle": 120.0,
            "max_angle": 175.0,
            "min_perimeter": 60.0,
            "max_jump": 25.0 
        }
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    saved_config = json.load(f)
                    self.config_data.update(saved_config)
                    
                    # Подгрузка сохраненной матрицы камеры
                    if "camera_matrix" in saved_config:
                        self.camera_matrix = np.array(saved_config["camera_matrix"], dtype=np.float32)
                    if "dist_coeffs" in saved_config:
                        self.dist_coeffs = np.array(saved_config["dist_coeffs"], dtype=np.float32)
            except Exception as e:
                print(f"Error loading config: {e}")

    def save_config(self):
        try:
            self.config_data["cam_index"] = self.cam_var.get()
            self.config_data["marker_size"] = self.marker_size_var.get()
            self.config_data["osc_ip"] = self.osc_ip_var.get()
            self.config_data["osc_port"] = self.osc_port_var.get()
            self.config_data["marker_ids"] = self.ids_var.get()
            self.config_data["min_angle"] = self.min_angle_var.get()
            self.config_data["max_angle"] = self.max_angle_var.get()
            self.config_data["min_perimeter"] = self.min_perim_var.get()
            self.config_data["max_jump"] = self.max_jump_var.get()
            
            # Сохранение матриц камеры в удобочитаемый JSON формат
            self.config_data["camera_matrix"] = self.camera_matrix.tolist()
            self.config_data["dist_coeffs"] = self.dist_coeffs.tolist()
            
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self.config_data, f, indent=4)
        except Exception as e:
            print(f"Error saving config: {e}")

    def create_widgets(self):
        left_frame = ctk.CTkFrame(self.root, width=340)
        left_frame.pack(side="left", fill="y", padx=12, pady=12)

        ctk.CTkLabel(left_frame, text="SlimeVR ArUco Tracker", 
                     font=ctk.CTkFont(size=22, weight="bold")).pack(pady=12)

        ctk.CTkLabel(left_frame, text="Camera Index:").pack(anchor="w", padx=15, pady=(4,0))
        self.cam_var = ctk.IntVar(value=self.config_data["cam_index"])
        ctk.CTkEntry(left_frame, textvariable=self.cam_var).pack(padx=15, pady=2, fill="x")

        ctk.CTkLabel(left_frame, text="Marker Size (mm):").pack(anchor="w", padx=15, pady=(4,0))
        self.marker_size_var = ctk.DoubleVar(value=self.config_data["marker_size"])
        ctk.CTkEntry(left_frame, textvariable=self.marker_size_var).pack(padx=15, pady=2, fill="x")

        ctk.CTkLabel(left_frame, text="OSC IP:").pack(anchor="w", padx=15, pady=(4,0))
        self.osc_ip_var = ctk.StringVar(value=self.config_data["osc_ip"])
        ctk.CTkEntry(left_frame, textvariable=self.osc_ip_var).pack(padx=15, pady=2, fill="x")

        ctk.CTkLabel(left_frame, text="OSC Port:").pack(anchor="w", padx=15, pady=(4,0))
        self.osc_port_var = ctk.IntVar(value=self.config_data["osc_port"])
        ctk.CTkEntry(left_frame, textvariable=self.osc_port_var).pack(padx=15, pady=2, fill="x")

        ctk.CTkLabel(left_frame, text="Marker IDs (comma separated):").pack(anchor="w", padx=15, pady=(4,0))
        self.ids_var = ctk.StringVar(value=self.config_data["marker_ids"])
        ctk.CTkEntry(left_frame, textvariable=self.ids_var).pack(padx=15, pady=2, fill="x")

        angle_frame = ctk.CTkFrame(left_frame, fg_color="transparent")
        angle_frame.pack(padx=15, pady=(8, 0), fill="x")

        min_frame = ctk.CTkFrame(angle_frame, fg_color="transparent")
        min_frame.pack(side="left", expand=True, fill="x", padx=(0, 5))
        ctk.CTkLabel(min_frame, text="Min Angle (deg):").pack(anchor="w")
        self.min_angle_var = ctk.DoubleVar(value=self.config_data["min_angle"])
        ctk.CTkEntry(min_frame, textvariable=self.min_angle_var).pack(fill="x")

        max_frame = ctk.CTkFrame(angle_frame, fg_color="transparent")
        max_frame.pack(side="left", expand=True, fill="x", padx=(5, 0))
        ctk.CTkLabel(max_frame, text="Max Angle (deg):").pack(anchor="w")
        self.max_angle_var = ctk.DoubleVar(value=self.config_data["max_angle"])
        ctk.CTkEntry(max_frame, textvariable=self.max_angle_var).pack(fill="x")

        filters_frame = ctk.CTkFrame(left_frame, fg_color="transparent")
        filters_frame.pack(padx=15, pady=(4, 2), fill="x")

        perim_frame = ctk.CTkFrame(filters_frame, fg_color="transparent")
        perim_frame.pack(side="left", expand=True, fill="x", padx=(0, 5))
        ctk.CTkLabel(perim_frame, text="Min Perim (px):").pack(anchor="w")
        self.min_perim_var = ctk.DoubleVar(value=self.config_data["min_perimeter"])
        ctk.CTkEntry(perim_frame, textvariable=self.min_perim_var).pack(fill="x")

        jump_frame = ctk.CTkFrame(filters_frame, fg_color="transparent")
        jump_frame.pack(side="left", expand=True, fill="x", padx=(5, 0))
        ctk.CTkLabel(jump_frame, text="Max Jump (deg):").pack(anchor="w")
        self.max_jump_var = ctk.DoubleVar(value=self.config_data["max_jump"])
        ctk.CTkEntry(jump_frame, textvariable=self.max_jump_var).pack(fill="x")

        btn_frame = ctk.CTkFrame(left_frame)
        btn_frame.pack(pady=20, padx=15, fill="x")

        self.start_btn = ctk.CTkButton(btn_frame, text="▶ START", 
                                       fg_color="green", hover_color="darkgreen", height=40, command=self.toggle_tracking)
        self.start_btn.pack(side="top", pady=2, fill="x")

        sub_btn_frame = ctk.CTkFrame(btn_frame, fg_color="transparent")
        sub_btn_frame.pack(side="top", pady=2, fill="x")

        self.preview_btn = ctk.CTkButton(sub_btn_frame, text="👁 Preview", 
                                         height=40, command=self.toggle_preview)
        self.preview_btn.pack(side="left", padx=(0,2), expand=True, fill="x")

        self.log_btn = ctk.CTkButton(sub_btn_frame, text="📝 REC LOG", 
                                     fg_color="purple", hover_color="darkmagenta", height=40, command=self.toggle_logging)
        self.log_btn.pack(side="left", padx=(2,2), expand=True, fill="x")

        # Новая кнопка калибровки
        self.calib_btn = ctk.CTkButton(sub_btn_frame, text="📷 CALIBRATE", 
                                       fg_color="#A5691F", hover_color="#8A5517", height=40, command=self.toggle_calibration)
        self.calib_btn.pack(side="left", padx=(2,0), expand=True, fill="x")

        self.status_label = ctk.CTkLabel(left_frame, text="● Stopped", 
                                         text_color="gray", font=ctk.CTkFont(size=15))
        self.status_label.pack(pady=5)

        self.log_text = ctk.CTkTextbox(self.root)
        self.log_text.pack(side="right", fill="both", expand=True, padx=(0,12), pady=12)

        if "camera_matrix" in self.config_data:
            self.log("Ready. Custom Camera Matrix Loaded.")
        else:
            self.log("Ready. Default Camera Matrix. Calibrate recommended.")

    def log(self, msg):
        self.log_text.insert("end", f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_text.see("end")

    def safe_log(self, msg):
        self.root.after(0, lambda: self.log(msg))

    def on_closing(self):
        self.running = False
        self.preview_running = False
        self.calib_running = False
        if self.is_logging and self.log_file:
            self.log_file.close()
        self.save_config()
        self.root.after(200, self.root.destroy)

    def toggle_logging(self):
        if not self.is_logging:
            timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"aruco_log_{timestamp_str}.csv"
            try:
                self.log_file = open(filename, 'w', newline='')
                self.csv_writer = csv.writer(self.log_file)
                self.csv_writer.writerow(["Timestamp", "MarkerID", "Qx", "Qy", "Qz", "Qw", "AOI_deg", "Delta_deg", "IsGlitch"])
                self.is_logging = True
                self.log_btn.configure(text="⏹ STOP LOG", fg_color="red", hover_color="darkred")
                self.log(f"Logging started: {filename}")
            except Exception as e:
                self.log(f"Error creating log file: {e}")
        else:
            self.is_logging = False
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.csv_writer = None
            self.log_btn.configure(text="📝 REC LOG", fg_color="purple", hover_color="darkmagenta")
            self.log("Logging stopped and saved.")

    # ====================== CALIBRATION LOGIC ======================
    def toggle_calibration(self):
        if self.running or self.preview_running:
            messagebox.showwarning("Warning", "Stop tracking and preview before calibrating.")
            return
        if not self.calib_running:
            self.start_calibration()
        else:
            self.calib_running = False

    def start_calibration(self):
        try:
            cam_index = int(self.cam_var.get())
        except ValueError:
            messagebox.showerror("Error", "Camera index must be an integer!")
            return

        self.calib_running = True
        self.calib_btn.configure(text="⏹ CANCEL CALIB", fg_color="red", hover_color="darkred")
        self.start_btn.configure(state="disabled")
        self.preview_btn.configure(state="disabled")
        
        self.log("Calibration started. Show ChArUco 5x7 board.")
        self.calib_thread = threading.Thread(target=self.calibration_loop, args=(cam_index,), daemon=True)
        self.calib_thread.start()

    def reset_calib_ui(self):
        self.calib_running = False
        self.calib_btn.configure(text="📷 CALIBRATE", fg_color="#A5691F", hover_color="#8A5517")
        self.start_btn.configure(state="normal")
        self.preview_btn.configure(state="normal")

    def calibration_loop(self, cam_index):
        cap = cv2.VideoCapture(cam_index)
        if not cap.isOpened():
            self.safe_log("❌ Error: Could not open camera.")
            self.root.after(0, self.reset_calib_ui)
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Конфигурация стандартной ChArUco доски
        # 5 столбцов, 7 строк, квадрат 40мм, маркер 30мм, словарь 4х4_50
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        board = cv2.aruco.CharucoBoard((5, 7), 0.035, 0.02625, dictionary)
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        all_charuco_corners = []
        all_charuco_ids = []

        self.safe_log("Press C to capture a frame. Capture ~15 frames.")
        self.safe_log("Press ENTER when done. Press ESC to cancel.")

        while self.calib_running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.03)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            c_corners, c_ids = None, None

            # Универсальный блок для OpenCV 4.6 и 4.7+
            try:
                charuco_detector = cv2.aruco.CharucoDetector(board)
                c_corners, c_ids, _, _ = charuco_detector.detectBoard(frame)
            except AttributeError:
                corners, ids, _ = detector.detectMarkers(gray)
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    _, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

            if c_corners is not None and c_ids is not None and len(c_corners) > 3:
                cv2.aruco.drawDetectedCornersCharuco(frame, c_corners, c_ids)

            cv2.putText(frame, f"Captures: {len(all_charuco_corners)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, "C - Capture | ENTER - Compute | ESC - Cancel", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow(self.CALIB_WIN_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('c') or key == ord('C') or key == ord('с'):
                if c_corners is not None and len(c_corners) > 4:
                    all_charuco_corners.append(c_corners)
                    all_charuco_ids.append(c_ids)
                    self.safe_log(f"Captured! Total frames: {len(all_charuco_corners)}")
                else:
                    self.safe_log("Not enough corners visible to capture.")

            elif key == 13:  # Клавиша ENTER
                if len(all_charuco_corners) < 5:
                    self.safe_log("Need at least 5 captures to calibrate!")
                else:
                    self.safe_log("Computing calibration... Please wait.")
                    try:
                        # Универсальный метод вычисления
                        obj_points, img_points = [], []
                        for corners, ids in zip(all_charuco_corners, all_charuco_ids):
                            try:
                                obj_pts, img_pts = board.matchImagePoints(corners, ids)
                                if len(obj_pts) > 3:
                                    obj_points.append(obj_pts)
                                    img_points.append(img_pts)
                            except AttributeError:
                                pass
                        
                        if len(obj_points) > 0:
                            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
                        else:
                            ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, board, gray.shape[::-1], None, None)
                        
                        self.camera_matrix = mtx
                        self.dist_coeffs = dist
                        self.save_config()
                        self.safe_log("✅ Calibration successful and saved to config!")
                    except Exception as e:
                        self.safe_log(f"❌ Calibration failed: {e}")
                self.calib_running = False
                break

            elif key == 27 or cv2.getWindowProperty(self.CALIB_WIN_NAME, cv2.WND_PROP_VISIBLE) < 1:
                self.safe_log("Calibration cancelled.")
                self.calib_running = False
                break

        cap.release()
        cv2.destroyAllWindows()
        self.root.after(0, self.reset_calib_ui)


    # ====================== ПРЕВЬЮ И ТРЕКИНГ ======================
    def toggle_preview(self):
        if self.running or self.calib_running:
            messagebox.showwarning("Warning", "Stop other processes before opening preview.")
            return
        if not self.preview_running:
            self.start_preview()
        else:
            self.stop_preview()

    def start_preview(self):
        try:
            cam_index = int(self.cam_var.get())
        except ValueError:
            messagebox.showerror("Error", "Camera index must be an integer!")
            return

        self.preview_running = True
        self.preview_btn.configure(text="⏹ Close", fg_color="orange", hover_color="darkorange")
        self.start_btn.configure(state="disabled")
        self.calib_btn.configure(state="disabled")
        self.log("Preview opened (3D Mode)")
        
        self.preview_thread = threading.Thread(target=self.preview_loop, args=(cam_index,), daemon=True)
        self.preview_thread.start()

    def stop_preview(self):
        self.preview_running = False

    def reset_preview_ui(self):
        self.preview_running = False
        self.preview_btn.configure(text="👁 Preview", fg_color=["#3B8ED0", "#1F6AA5"], hover_color=["#3279B2", "#144870"])
        self.start_btn.configure(state="normal")
        self.calib_btn.configure(state="normal")

    def toggle_tracking(self):
        if self.preview_running or self.calib_running:
            messagebox.showwarning("Warning", "Close preview/calib before starting tracking.")
            return
        if not self.running:
            self.start_tracking()
        else:
            self.stop_tracking()

    def start_tracking(self):
        try:
            cam_idx = int(self.cam_var.get())
            port = int(self.osc_port_var.get())
            ip = self.osc_ip_var.get()
        except ValueError:
            messagebox.showerror("Error", "Check data types (port and camera index)!")
            return

        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            messagebox.showerror("Error", f"Could not open camera {cam_idx}")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        try:
            self.osc_client = udp_client.SimpleUDPClient(ip, port)
        except Exception as e:
            messagebox.showerror("OSC Error", f"Failed to initialize OSC client: {e}")
            self.cap.release()
            return

        self.marker_buffers.clear()
        self.last_ui_log_time.clear()

        self.running = True
        self.start_btn.configure(text="⏹ STOP", fg_color="red", hover_color="darkred")
        self.preview_btn.configure(state="disabled")
        self.calib_btn.configure(state="disabled")
        self.status_label.configure(text="● Running", text_color="lime")

        self.thread = threading.Thread(target=self.tracking_loop, daemon=True)
        self.thread.start()
        self.log("Tracking thread spawned (Headless mode)")

    def stop_tracking(self):
        self.running = False
        self.start_btn.configure(text="▶ START", fg_color="green", hover_color="darkgreen")
        self.preview_btn.configure(state="normal")
        self.calib_btn.configure(state="normal")
        self.status_label.configure(text="● Stopped", text_color="gray")
        self.log("Stopping tracking thread...")

    # ====================== ЯДРО ОБРАБОТКИ ======================
    def _core_process_marker(self, mid, corner, rvec, tvec, current_time, allowed_ids, marker_size_meters, is_preview, frame=None):
        if allowed_ids is not None and mid not in allowed_ids:
            return None, None, True

        min_a = self.min_angle_var.get()
        max_a = self.max_angle_var.get()
        min_p = self.min_perim_var.get()
        max_j = self.max_jump_var.get()

        perimeter = cv2.arcLength(corner[0], True)
        if perimeter < min_p:
            return "TOO FAR", (0, 165, 255), True

        R, _ = cv2.Rodrigues(rvec)
        cos_aoi = max(-1.0, min(1.0, R[2, 2]))
        angle_deg = math.acos(cos_aoi) * (180.0 / math.pi)

        if not (min_a <= angle_deg <= max_a):
            return "ANGLE BLOCKED", (0, 0, 255), True

        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
        current_q = np.array([qx, qy, qz, qw])

        if mid not in self.marker_buffers:
            self.marker_buffers[mid] = {"queue": deque(maxlen=10), "last_time": 0.0, "drop_count": 0}
        buf = self.marker_buffers[mid]

        if buf["last_time"] > 0.0 and (current_time - buf["last_time"] > 0.5):
            buf["queue"].clear()
            buf["drop_count"] = 0

        if len(buf["queue"]) > 0:
            if np.dot(current_q, buf["queue"][-1]) < 0:
                current_q = -current_q

        is_glitch = False
        delta_angle_deg = 0.0

        if len(buf["queue"]) > 0:
            recent_qs = list(buf["queue"])[-5:]
            q_avg_recent = np.mean(recent_qs, axis=0)
            norm = np.linalg.norm(q_avg_recent)
            if norm > 1e-6:
                q_avg_recent /= norm
            else:
                q_avg_recent = current_q
            
            dot_p = np.clip(np.abs(np.dot(q_avg_recent, current_q)), 0.0, 1.0)
            delta_angle_deg = 2.0 * math.acos(dot_p) * (180.0 / math.pi)

            if delta_angle_deg > max_j:
                buf["drop_count"] += 1
                if buf["drop_count"] < 5:
                    is_glitch = True
                else:
                    buf["queue"].clear()
                    buf["drop_count"] = 0
            else:
                buf["drop_count"] = 0

        if self.is_logging and self.csv_writer:
            self.csv_writer.writerow([
                f"{current_time:.4f}", mid,
                f"{current_q[0]:.4f}", f"{current_q[1]:.4f}", f"{current_q[2]:.4f}", f"{current_q[3]:.4f}",
                f"{angle_deg:.2f}", f"{delta_angle_deg:.2f}", 1 if is_glitch else 0
            ])

        if is_glitch:
            buf["last_time"] = current_time
            return f"GLITCH ({delta_angle_deg:.0f}*)", (0, 255, 255), True

        buf["queue"].append(current_q)
        buf["last_time"] = current_time

        if not is_preview and len(buf["queue"]) == 10:
            q_final = np.mean(list(buf["queue"]), axis=0)
            q_final /= np.linalg.norm(q_final)
            
            dots = [np.abs(np.dot(q, q_final)) for q in buf["queue"]]
            is_stable = 1 if min(dots) > 0.995 else 0
            timestamp_ms = int(current_time * 1000)

            self.osc_client.send_message(
                '/aruco/correction', 
                [mid, float(q_final[0]), float(q_final[1]), float(q_final[2]), float(q_final[3]), timestamp_ms, is_stable]
            )

            if current_time - self.last_ui_log_time.get(mid, 0) > 1.5:
                self.safe_log(f"⚡ OSC Sent ID {mid} | Angle: {angle_deg:.1f}° | Stable: {is_stable}")
                self.last_ui_log_time[mid] = current_time

        if is_preview and frame is not None:
            cube_color = (0, 255, 0)
            r = marker_size_meters / 2.0
            cube_vertices = np.array([
                [-r, -r, 0],  [r, -r, 0],  [r, r, 0],  [-r, r, 0],
                [-r, -r, marker_size_meters], [r, -r, marker_size_meters], 
                [r, r, marker_size_meters], [-r, r, marker_size_meters]
            ], dtype=np.float32)

            img_pts, _ = cv2.projectPoints(cube_vertices, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            img_pts = np.int32(img_pts).reshape(-1, 2)

            cv2.drawContours(frame, [img_pts[:4]], -1, cube_color, 2)
            for start, end in zip(range(4), range(4, 8)):
                cv2.line(frame, tuple(img_pts[start]), tuple(img_pts[end]), cube_color, 2)
            cv2.drawContours(frame, [img_pts[4:]], -1, cube_color, 2)

            text_coord = tuple(np.int32(corner[0][0]))
            y_offset = text_coord[1] - 15
            
            cv2.putText(frame, f"ID: {mid} [OSC VALID]", (text_coord[0], y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, cube_color, 2)
            cv2.putText(frame, f"Angle: {angle_deg:.1f} | Delta: {delta_angle_deg:.1f}", (text_coord[0], y_offset + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255),  1)
            cv2.putText(frame, f"Perim: {perimeter:.1f} px", (text_coord[0], y_offset + 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        return "OSC VALID", (0, 255, 0), False

    # ====================== ПОТОКИ ======================
    def preview_loop(self, cam_index):
        cap = cv2.VideoCapture(cam_index)
        if not cap.isOpened():
            self.safe_log(f"❌ Error: Could not open camera {cam_index}")
            self.root.after(0, self.reset_preview_ui)
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.marker_buffers.clear()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        while self.preview_running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.03)
                continue

            current_time = time.time()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                marker_size_meters = self.marker_size_var.get() / 1000.0
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_size_meters, self.camera_matrix, self.dist_coeffs
                )
                
                for i in range(len(ids)):
                    mid = int(ids[i][0])
                    status_text, cube_color, should_skip = self._core_process_marker(
                        mid, corners[i], rvecs[i], tvecs[i], current_time, 
                        allowed_ids=None, marker_size_meters=marker_size_meters, is_preview=True, frame=frame
                    )
                    
                    if should_skip and status_text:
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        text_coord = tuple(np.int32(corners[i][0][0]))
                        cv2.putText(frame, f"ID: {mid} [{status_text}]", (text_coord[0], text_coord[1] - 15), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, cube_color, 2)

            cv2.imshow(self.WIN_NAME, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27 or cv2.getWindowProperty(self.WIN_NAME, cv2.WND_PROP_VISIBLE) < 1:
                self.preview_running = False
                break

        cap.release()
        cv2.destroyAllWindows()
        self.root.after(0, self.reset_preview_ui)
        self.safe_log("Preview closed")

    def tracking_loop(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        try:
            marker_ids = [int(x.strip()) for x in self.ids_var.get().split(',') if x.strip()]
        except ValueError:
            self.safe_log("❌ ERROR: Invalid Marker IDs format.")
            self.running = False
            return

        marker_size_meters = self.marker_size_var.get() / 1000.0

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.03)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                current_time = time.time()
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_size_meters, self.camera_matrix, self.dist_coeffs
                )
                
                for i in range(len(ids)):
                    mid = int(ids[i][0])
                    self._core_process_marker(
                        mid, corners[i], rvecs[i], tvecs[i], current_time, 
                        allowed_ids=marker_ids, marker_size_meters=marker_size_meters, is_preview=False
                    )

            time.sleep(0.005)

        if self.cap:
            self.cap.release()
            self.cap = None
        self.safe_log("Tracking thread terminated safely.")

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = ArUcoTrackerApp()
    app.run()