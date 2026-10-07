"""
SCARA Dynamic Conveyor Interception - Multithreaded Zero-Delay Controller
(Features: Y/N Homing Trigger, Standby Button, True Parallel Vision)
"""
import math
import time
import os
import cv2
import numpy as np
import serial
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple
from ultralytics import YOLO

# =============================================================================
# 1. HARDWARE, PORTS & MODEL
# =============================================================================
ESP32_PORT = "COM10"
ARDUINO_PORT = "COM9"
BAUDRATE = 115200
CAMERA_INDEX = 0
MODEL_PATH = r"D:\trialscara\runs\detect\ball_model\weights\best.pt"

L1_MM = 290.0
L2_MM = 180.0

Z_HIGH_DEG = 0.0
Z_LOW_DEG = -350.0

JAW_OPEN = 0.0
JAW_CLOSED = 64.0
JAW_SMALL = 102.0

HOME_J1_DEG = 0.0
HOME_J2_DEG = 0.0

STANDBY_X_MM = 0.0
STANDBY_Y_MM = 470.0
IR_OBJECT_START_X_MM = 300.0
IR_OBJECT_START_Y_MM = 400.0

DROP_BLUE = (-200.0, 200.0)
DROP_GREEN = (-300.0, 200.0)
DROP_YELLOW = (-400.0, 200.0)
DROP_LARGE = (-200.0, 200.0)
DROP_SMALL = (-300.0, 200.0)
DROP_DEFAULT = (-200.0, 200.0)

ROI_X_MIN = 200
ROI_X_MAX = 440

CONVEYOR_SPEED_MM_PER_SEC = 34.0
TIMING_OFFSET_SEC = -0.45
SEARCH_X_MAX_MM = 245.0
SEARCH_X_MIN_MM = -245.0

STEPS_PER_DEG = 3200.0 / 360.0
M1_MULT, M1_MAX_V, M1_ACCEL = 3.0, 1000.0, 500.0
M2_MULT, M2_MAX_V, M2_ACCEL = 3.0, 1000.0, 500.0
M3_MULT, M3_MAX_V, M3_ACCEL = 1.0, 1000.0, 500.0
M4_MULT, M4_MAX_V, M4_ACCEL = 3.0, 600.0, 300.0
ARDUINO_DONE_DELAY_SEC = 0.25


# =============================================================================
# Shared Memory / State variables for Threads
# =============================================================================
class SharedState:
    def __init__(self):
        self.last_seen_obj = None
        self.sort_mode = None
        self.running = True
        self.state = "ASK_HOMING"
        self.lock = threading.Lock()


shared_state = SharedState()


def calculate_inverse_kinematics_mm(x_mm: float, y_mm: float) -> Optional[Tuple[float, float]]:
    r = math.sqrt(x_mm ** 2 + y_mm ** 2)
    if r > (L1_MM + L2_MM) or r < abs(L1_MM - L2_MM): return None
    cos_angle2 = (r ** 2 - L1_MM ** 2 - L2_MM ** 2) / (2.0 * L1_MM * L2_MM)
    cos_angle2 = max(-1.0, min(1.0, cos_angle2))
    angle2 = math.acos(cos_angle2)
    k1 = L1_MM + L2_MM * math.cos(angle2)
    k2 = L2_MM * math.sin(angle2)
    angle1 = math.atan2(y_mm, x_mm) - math.atan2(k2, k1)
    q1_deg = math.degrees(angle1)
    q2_deg = -math.degrees(angle2)
    return q1_deg, q2_deg


def estimate_arm_move_time(start_j1, start_j2, start_z, end_j1, end_j2, end_z) -> float:
    def t(delta, mult, max_v, acc):
        steps = abs(round(delta * STEPS_PER_DEG * mult))
        if steps == 0: return 0.0
        s2v = (max_v ** 2) / (2.0 * acc)
        return 2.0 * math.sqrt(steps / acc) if steps <= 2.0 * s2v else (steps / max_v) + (max_v / acc)

    start_j4 = start_j2 - start_j1
    end_j4 = end_j2 - end_j1
    t1 = t(end_j1 - start_j1, M1_MULT, M1_MAX_V, M1_ACCEL)
    t2 = t(end_j2 - start_j2, M2_MULT, M2_MAX_V, M2_ACCEL)
    t3 = t(end_z - start_z, M3_MULT, M3_MAX_V, M3_ACCEL)
    t4 = t(end_j4 - start_j4, M4_MULT, M4_MAX_V, M4_ACCEL)
    return max(t1, t2, t3, t4)


@dataclass
class MoveCommand:
    label: str;
    j1_deg: float;
    j2_deg: float;
    z_deg: float;
    jaw_deg: float;
    pre_wait_sec: float = 0.0

    def to_serial(self) -> str: return f"MOVE,{self.j1_deg:.2f},{self.j2_deg:.2f},{self.z_deg:.2f},{self.jaw_deg:.2f}\n"


class DynamicInterceptionController:
    def __init__(self, ard_conn, esp_conn):
        self.ard_conn, self.esp_conn = ard_conn, esp_conn
        self.state = "IDLE"
        self.initial_standby_done = False
        self.current_j1, self.current_j2, self.current_z, self.current_jaw = HOME_J1_DEG, HOME_J2_DEG, Z_HIGH_DEG, JAW_OPEN
        self.command_queue, self.active_command = [], None
        self.cmd_start_time, self.cmd_expected_duration = 0.0, 0.0
        self.pre_delay_end_time = 0.0

    def enqueue_command(self, cmd: MoveCommand):
        self.command_queue.append(cmd)
        if self.state == "IDLE": self.send_next_command()

    def get_arm_availability(self):
        now = time.time();
        remaining_time = 0.0
        sim_j1, sim_j2, sim_z = self.current_j1, self.current_j2, self.current_z

        if self.state == "PRE_WAIT" and self.active_command:
            remaining_time += max(0.0, self.pre_delay_end_time - now) + estimate_arm_move_time(sim_j1, sim_j2, sim_z,
                                                                                               self.active_command.j1_deg,
                                                                                               self.active_command.j2_deg,
                                                                                               self.active_command.z_deg) + ARDUINO_DONE_DELAY_SEC
            sim_j1, sim_j2, sim_z = self.active_command.j1_deg, self.active_command.j2_deg, self.active_command.z_deg
        elif self.state == "WAITING_DONE" and self.active_command:
            remaining_time += max(0.0, self.cmd_expected_duration - (now - self.cmd_start_time))
            sim_j1, sim_j2, sim_z = self.active_command.j1_deg, self.active_command.j2_deg, self.active_command.z_deg

        for cmd in self.command_queue:
            remaining_time += cmd.pre_wait_sec + estimate_arm_move_time(sim_j1, sim_j2, sim_z, cmd.j1_deg, cmd.j2_deg,
                                                                        cmd.z_deg) + ARDUINO_DONE_DELAY_SEC
            sim_j1, sim_j2, sim_z = cmd.j1_deg, cmd.j2_deg, cmd.z_deg
        return remaining_time, sim_j1, sim_j2, sim_z

    def handle_ir_trigger(self, jaw_to_use: float, drop_x: float, drop_y: float) -> None:
        if not self.initial_standby_done: return

        busy_time, start_j1, start_j2, start_z = self.get_arm_availability()
        print(f"\n🚨 IR DETECTED! Target Drop: ({drop_x}, {drop_y}) | Jaw: {jaw_to_use}°")

        best_x, best_ik, best_diff, best_t_obj, best_t_arm = None, None, float("inf"), 0.0, 0.0

        for x_cand in np.linspace(SEARCH_X_MAX_MM, SEARCH_X_MIN_MM, 491):
            ik = calculate_inverse_kinematics_mm(float(x_cand), IR_OBJECT_START_Y_MM)
            if not ik: continue
            t_obj = ((IR_OBJECT_START_X_MM - float(x_cand)) / CONVEYOR_SPEED_MM_PER_SEC) + TIMING_OFFSET_SEC
            t_arm = busy_time + estimate_arm_move_time(start_j1, start_j2, start_z, ik[0], ik[1], Z_LOW_DEG)
            if t_obj >= t_arm:
                diff = t_obj - t_arm
                if diff < best_diff:
                    best_diff, best_x, best_ik, best_t_obj, best_t_arm = diff, float(x_cand), ik, t_obj, t_arm

        if not best_x or not best_ik:
            print("❌ Object too fast or out of range!")
            return

        extra_wait = max(0.0, best_t_obj - best_t_arm)
        print(f"🛠️  INTERCEPT POINT: Pick from Conveyor at X = {best_x:.1f} mm")
        print(f"⏱️  Sync Wait: {extra_wait:.2f}s")

        drop_ik = calculate_inverse_kinematics_mm(drop_x, drop_y)
        if not drop_ik:
            print("❌ Error: Drop coordinate unreachable!")
            return

        self.enqueue_command(MoveCommand(f"1. Pick at X={best_x:.0f}", best_ik[0], best_ik[1], Z_LOW_DEG, jaw_to_use,
                                         pre_wait_sec=extra_wait))
        self.enqueue_command(MoveCommand("2. Lift to Top", best_ik[0], best_ik[1], Z_HIGH_DEG, jaw_to_use))
        self.enqueue_command(
            MoveCommand(f"3. Drop at ({drop_x:.0f}, {drop_y:.0f})", drop_ik[0], drop_ik[1], Z_HIGH_DEG, JAW_OPEN))

    def send_next_command(self):
        if not self.command_queue:
            self.active_command = None;
            self.state = "IDLE"
            if not self.initial_standby_done:
                self.initial_standby_done = True
                print("\n✅ Arm set at (0, 470)! Select Mode on Camera window to Start Conveyor.")
            return

        self.active_command = self.command_queue.pop(0)
        if self.active_command.pre_wait_sec > 0:
            self.state = "PRE_WAIT"
            self.pre_delay_end_time = time.time() + self.active_command.pre_wait_sec
            return
        self._dispatch_active_command()

    def _dispatch_active_command(self):
        if not self.active_command: return
        move_t = estimate_arm_move_time(self.current_j1, self.current_j2, self.current_z, self.active_command.j1_deg,
                                        self.active_command.j2_deg, self.active_command.z_deg)
        self.cmd_start_time, self.cmd_expected_duration = time.time(), move_t + ARDUINO_DONE_DELAY_SEC
        msg = self.active_command.to_serial()
        self.ard_conn.write(msg.encode("utf-8"))
        print(f"PYTHON -> {self.active_command.label} (Est: {move_t:.2f}s)")
        self.state = "WAITING_DONE"

    def update(self):
        if self.state == "PRE_WAIT" and time.time() >= self.pre_delay_end_time: self._dispatch_active_command()
        if self.ard_conn.in_waiting > 0:
            line = self.ard_conn.readline().decode("utf-8", errors="ignore").strip()
            if line == "DONE" and self.active_command:
                self.current_j1, self.current_j2, self.current_z, self.current_jaw = self.active_command.j1_deg, self.active_command.j2_deg, self.active_command.z_deg, self.active_command.jaw_deg
                self.active_command = None
                self.send_next_command()
            elif line == "HOMED_DONE":
                self.current_j1, self.current_j2, self.current_z = HOME_J1_DEG, HOME_J2_DEG, Z_HIGH_DEG
                print("\n✅ Homing Complete! Arm is at (0,0).")
                with shared_state.lock:
                    shared_state.state = "WAIT_FOR_S"


def vision_worker():
    threshold_px = 45.0
    if os.path.exists("threshold.txt"):
        with open("threshold.txt", "r") as f:
            threshold_px = float(f.read().strip())
        print(f"\n✅ SUCCESS: Loaded Threshold from TXT -> {threshold_px} px")
    else:
        print(f"\n⚠️ WARNING: threshold.txt NOT FOUND! Using default -> {threshold_px} px")

    print("🔥 Warming up YOLO model...")
    model = YOLO(MODEL_PATH)
    model(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
    print("✅ Model Ready!")

    cap = cv2.VideoCapture(CAMERA_INDEX)

    while shared_state.running:
        ret, frame = cap.read()
        if not ret: break
        H, W, _ = frame.shape

        cv2.line(frame, (ROI_X_MIN, 0), (ROI_X_MIN, H), (255, 0, 0), 2)
        cv2.line(frame, (ROI_X_MAX, 0), (ROI_X_MAX, H), (255, 0, 0), 2)

        roi_mask = np.zeros(frame.shape, dtype=np.uint8)
        roi_mask[:, ROI_X_MIN:ROI_X_MAX] = frame[:, ROI_X_MIN:ROI_X_MAX]

        results = model(roi_mask, conf=0.5, verbose=False)

        largest_radius = 0
        detected_obj = None

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            radius = ((x2 - x1) + (y2 - y1)) / 4.0
            color = model.names[int(box.cls[0])].lower()
            size = "LARGE" if radius >= threshold_px else "SMALL"

            label = f"{color}_{size} (R:{radius:.1f})"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            MARGIN = 5
            is_fully_inside = (x1 > ROI_X_MIN + MARGIN) and (x2 < ROI_X_MAX - MARGIN)

            if is_fully_inside:
                if radius > largest_radius:
                    largest_radius = radius
                    detected_obj = {'color': color, 'size': size}

        with shared_state.lock:
            if detected_obj:
                shared_state.last_seen_obj = detected_obj
            current_mode = shared_state.sort_mode
            current_state = shared_state.state

        # 🎯 FIX: Updated UI text to show the 'n' skip option
        if current_state == "ASK_HOMING":
            cv2.putText(frame, "Start Homing? 'y' (or 'n' to skip)", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2)
        elif current_state == "HOMING_IN_PROGRESS":
            cv2.putText(frame, "HOMING... PLEASE WAIT", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        elif current_state == "WAIT_FOR_S":
            cv2.putText(frame, "Press 's' to Move to Standby (0,470)", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)
        elif current_state in ["ASK_MODE", "RUNNING"]:
            mode_str = current_mode.upper() if current_mode else "NONE"
            if current_state == "ASK_MODE":
                cv2.putText(frame, "Mode: Color(c) / Size(s)? [h=Home, b=Standby]", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)
            else:
                cv2.putText(frame, f"RUNNING | Mode: {mode_str} | [h=Home, b=Standby]", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("SCARA Dashboard", frame)
        key = cv2.waitKey(1) & 0xFF

        with shared_state.lock:
            if key == 27:
                shared_state.running = False
                break
            shared_state.last_key = key

    cap.release()
    cv2.destroyAllWindows()


def main():
    try:
        ard = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=0.01)
        esp = serial.Serial(ESP32_PORT, BAUDRATE, timeout=0.01)
    except Exception as e:
        print(f"Port Error: {e}")
        return

    time.sleep(2)
    esp.write(b"STOP\n")
    ard.reset_input_buffer()
    esp.reset_input_buffer()

    ctrl = DynamicInterceptionController(ard, esp)

    vision_thread = threading.Thread(target=vision_worker)
    vision_thread.start()

    while shared_state.running:
        ctrl.update()

        key = None
        with shared_state.lock:
            key = getattr(shared_state, 'last_key', None)
            shared_state.last_key = None

        if key:
            if key == ord('h'):
                print("\n🏠 Home Button Pressed! Returning to Limit Switches (0,0)...")
                esp.write(b"STOP\n")
                ctrl.command_queue.clear()
                if ctrl.current_z < -5.0:
                    ctrl.enqueue_command(
                        MoveCommand("Lift for Home", ctrl.current_j1, ctrl.current_j2, Z_HIGH_DEG, JAW_OPEN))
                ctrl.enqueue_command(MoveCommand("Return to Home", HOME_J1_DEG, HOME_J2_DEG, Z_HIGH_DEG, JAW_OPEN))
                with shared_state.lock:
                    shared_state.state = "ASK_HOMING"

            else:
                with shared_state.lock:
                    current_state = shared_state.state

                if current_state == "ASK_HOMING":
                    if key == ord('y'):
                        print("\n▶️ 'y' Pressed! Starting Homing Sequence...")
                        ard.write(b"HOME_LIMIT\n")
                        with shared_state.lock:
                            shared_state.state = "HOMING_IN_PROGRESS"
                    # 🎯 FIX: 'n' button logic to skip homing
                    elif key == ord('n'):
                        print("\n⏩ 'n' Pressed! Skipping Homing (Assuming arm is already at 0,0)...")
                        with shared_state.lock:
                            shared_state.state = "WAIT_FOR_S"

                elif current_state == "WAIT_FOR_S":
                    if key == ord('s'):
                        print("\n▶️ 's' Pressed! Moving to Standby Position (0, 470)...")
                        stby_ik = calculate_inverse_kinematics_mm(STANDBY_X_MM, STANDBY_Y_MM)
                        if stby_ik:
                            ctrl.enqueue_command(
                                MoveCommand("Setup: Move to Standby", stby_ik[0], stby_ik[1], Z_HIGH_DEG, JAW_OPEN))
                        with shared_state.lock:
                            shared_state.state = "ASK_MODE"

                elif current_state in ["ASK_MODE", "RUNNING"]:
                    if key == ord('b'):
                        print("\n🔙 'b' Pressed! Returning to Standby (0,470)...")
                        esp.write(b"STOP\n")
                        ctrl.command_queue.clear()
                        if ctrl.current_z < -5.0:
                            ctrl.enqueue_command(
                                MoveCommand("Lift for Standby", ctrl.current_j1, ctrl.current_j2, Z_HIGH_DEG, JAW_OPEN))
                        stby_ik = calculate_inverse_kinematics_mm(STANDBY_X_MM, STANDBY_Y_MM)
                        if stby_ik:
                            ctrl.enqueue_command(
                                MoveCommand("Return to Standby", stby_ik[0], stby_ik[1], Z_HIGH_DEG, JAW_OPEN))
                        with shared_state.lock:
                            shared_state.state = "ASK_MODE"

                    elif current_state == "ASK_MODE" and key in (ord('c'), ord('s')):
                        with shared_state.lock:
                            shared_state.sort_mode = chr(key)
                            shared_state.state = "RUNNING"
                        esp.reset_input_buffer()
                        esp.write(b"START\n")
                        print(f"\n▶️ Conveyor STARTED! Mode Selected: {chr(key).upper()}")

        if esp.in_waiting > 0:
            esp_line = esp.readline().decode("utf-8", errors="ignore").strip()
            with shared_state.lock:
                current_state = shared_state.state

            if esp_line == "OBJECT_DETECTED" and current_state == "RUNNING":
                jaw_to_use = JAW_CLOSED
                drop_target = DROP_DEFAULT

                with shared_state.lock:
                    current_obj = shared_state.last_seen_obj
                    current_mode = shared_state.sort_mode
                    shared_state.last_seen_obj = None

                mode_name = "COLOR (C)" if current_mode == 'c' else "SIZE (S)"

                if current_obj:
                    jaw_to_use = JAW_SMALL if current_obj['size'] == "SMALL" else JAW_CLOSED
                    if current_mode == 'c':
                        if current_obj['color'] == "blue":
                            drop_target = DROP_BLUE
                        elif current_obj['color'] == "green":
                            drop_target = DROP_GREEN
                        elif current_obj['color'] == "yellow":
                            drop_target = DROP_YELLOW
                    else:
                        drop_target = DROP_LARGE if current_obj['size'] == "LARGE" else DROP_SMALL
                    print("\n" + "⭐" * 25)
                    print(f"📦 NEW OBJECT READY FOR PICKUP!")
                    print(f"⚙️  Sorting Mode : {mode_name}")
                    print(f"🏷️  Object Type  : {current_obj['color'].upper()} - {current_obj['size']}")
                    print(f"🎯 Drop Target  : X={drop_target[0]}, Y={drop_target[1]}")
                    print("⭐" * 25)
                else:
                    print("\n" + "⚠️" * 15)
                    print("YOLO MISSED! Using Default Drop.")
                    print("⚠️" * 15)

                ctrl.handle_ir_trigger(jaw_to_use, drop_target[0], drop_target[1])

        time.sleep(0.001)

    vision_thread.join()
    try:
        esp.write(b"STOP\n")
    except:
        pass
    if ctrl.active_command or ctrl.current_z < -5.0:
        ard.write(f"MOVE,{ctrl.current_j1:.2f},{ctrl.current_j2:.2f},{Z_HIGH_DEG:.2f},{JAW_OPEN:.2f}\n".encode())
    ard.close()
    esp.close()


if __name__ == "__main__": main()