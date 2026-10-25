# ==============================================================================
# 体験型エンタメ ～海陸両用空母ロボット～ 統合制御サーバー (server.py)
# 【ロック廃除・速度700-1600(100刻み)・Tello中央撮影ボタン＆右上日時対応版】
# ==============================================================================

import os
import sys

os.environ["OPENCV_VIDEOIO_PRIORITY_GSTREAMER"] = "0"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay"
os.environ["GST_DISABLE_REGISTRY_FORK"] = "1"
os.environ["LIBCAMERA_LOG_LEVELS"] = "*:ERROR"
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

import time
import threading
import cv2
import serial
import glob
import datetime
import math
from collections import deque
import numpy as np
import RPi.GPIO as GPIO
from flask import Flask, render_template, jsonify, Response, request, send_from_directory

# Tello ライブラリ
try:
    from djitellopy import Tello
    HAS_TELLO_LIB = True
except ImportError:
    HAS_TELLO_LIB = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
KNOWN_FACES_DIR = os.path.join(BASE_DIR, 'known_faces')
SNAPSHOTS_DIR = os.path.join(BASE_DIR, 'snapshots')
os.makedirs(KNOWN_FACES_DIR, exist_ok=True)
os.makedirs(SNAPSHOTS_DIR, exist_ok=True)

# --- 甲板カメラの照準中心オフセット ---
DECK_OFFSET_X = 0
DECK_OFFSET_Y = 0

# --- ハードウェアモジュール初期化 ---
sys.path.append(os.path.expanduser('~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server'))
car_motor = None
car_buzzer = None
car_adc = None
pwm = None

try:
    from motor import Ordinary_Car
    from buzzer import Buzzer
    from adc import ADC
    from pca9685 import PCA9685

    car_motor = Ordinary_Car()
    car_motor.set_motor_model(0, 0, 0, 0)
    car_buzzer = Buzzer()
    car_adc = ADC()
    pwm = PCA9685()
    pwm.set_pwm_freq(50)
except Exception as e:
    print(f"モジュール読み込み警告: {e}")

motor_lock = threading.Lock()
current_speed = 800  # デフォルト800 (範囲: 700〜1600, 100刻み)
face_threshold = 0.50
brake_distance_cm = 25.0
user_custom_brake_dist = 25.0

def set_car_move(direction, speed=None):
    if not car_motor:
        return
    s = int(speed) if speed is not None else current_speed
    with motor_lock:
        try:
            if direction == 'forward':
                car_motor.set_motor_model(-s, -s, -s, -s)
            elif direction == 'backward':
                car_motor.set_motor_model(s, s, s, s)
            elif direction == 'slide_left':
                car_motor.set_motor_model(s, -s, -s, s)
            elif direction == 'slide_right':
                car_motor.set_motor_model(-s, s, s, -s)
            elif direction == 'slide_forward_left':
                car_motor.set_motor_model(0, -s, -s, 0)
            elif direction == 'slide_forward_right':
                car_motor.set_motor_model(-s, 0, 0, -s)
            elif direction == 'slide_backward_left':
                car_motor.set_motor_model(s, 0, 0, s)
            elif direction == 'slide_backward_right':
                car_motor.set_motor_model(0, s, 0, -s)
            elif direction == 'turn_left':
                car_motor.set_motor_model(s, s, -s, -s)
            elif direction == 'turn_right':
                car_motor.set_motor_model(-s, -s, s, s)
            elif direction == 'stop':
                car_motor.set_motor_model(0, 0, 0, 0)
        except Exception:
            pass

def set_screw_motors(power):
    if pwm:
        try:
            duty = int((power / 100.0) * 4095)
            pwm.set_pwm(13, 0, duty)
        except Exception:
            pass

set_car_move('stop')
set_screw_motors(0)

def trigger_snapshot_buzzer():
    def _worker():
        if car_buzzer:
            try:
                car_buzzer.set_state(True)
                time.sleep(0.03)
                car_buzzer.set_state(False)
            except Exception:
                pass
    threading.Thread(target=_worker, daemon=True).start()

# --- 超音波センサー制御 ---
TRIG_PIN = 27
ECHO_PIN = 22
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

latest_distance = 999.0
dist_lock = threading.Lock()
current_moving_direction = 'stop'

def ultrasonic_worker():
    global latest_distance, current_action_status
    while True:
        try:
            GPIO.output(TRIG_PIN, GPIO.LOW)
            time.sleep(0.01)
            GPIO.output(TRIG_PIN, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(TRIG_PIN, GPIO.LOW)

            timeout = time.time() + 0.03
            while GPIO.input(ECHO_PIN) == 0:
                if time.time() > timeout: break
            t1 = time.time()

            timeout = time.time() + 0.03
            while GPIO.input(ECHO_PIN) == 1:
                if time.time() > timeout: break
            t2 = time.time()

            d = (t2 - t1) * 17150
            if 0 < d < 400:
                with dist_lock:
                    latest_distance = d
                
                if d <= brake_distance_cm and current_moving_direction in ['forward', 'slide_forward_left', 'slide_forward_right']:
                    set_car_move('stop')
                    current_action_status = f"⚠️ 障害物検知 ({d:.1f}cm <= {brake_distance_cm:.0f}cm)：自動ブレーキ作動！前進はロック中"
        except Exception:
            pass
        time.sleep(0.05)

threading.Thread(target=ultrasonic_worker, daemon=True).start()

def get_safe_distance():
    with dist_lock:
        return latest_distance

# --- バッテリー電圧計測 ---
voltage_history = []
voltage_lock = threading.Lock()

def get_car_battery_percentage():
    with voltage_lock:
        if len(voltage_history) > 0:
            avg_v = sum(voltage_history) / len(voltage_history)
            soc = int((avg_v - 7.0) / (8.4 - 7.0) * 100)
            return max(0, min(100, soc))
    return 80

def battery_sampling_loop():
    global voltage_history
    while True:
        if car_adc:
            try:
                val = car_adc.read_adc(2) * (3 if getattr(car_adc, 'pcb_version', 2) == 1 else 2)
                if 5.0 <= val <= 9.0:
                    with voltage_lock:
                        voltage_history.append(val)
                        if len(voltage_history) > 10:
                            voltage_history.pop(0)
            except Exception:
                pass
        time.sleep(1.0)

threading.Thread(target=battery_sampling_loop, daemon=True).start()

# --- 顔認識 & テンプレート照合 ---
cascade_candidates = [
    'haarcascade_frontalface_default.xml',
    '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
    '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
]
face_cascade = cv2.CascadeClassifier()
for p in cascade_candidates:
    if os.path.exists(p):
        face_cascade.load(p)
        if not face_cascade.empty():
            break

user01_sample_path = os.path.join(KNOWN_FACES_DIR, 'USER01.jpg')
user01_loaded = False
user01_template = None
user01_hist = None

def reload_user_template():
    global user01_loaded, user01_template, user01_hist
    if os.path.exists(user01_sample_path):
        img = cv2.imread(user01_sample_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            user01_template = cv2.resize(img, (100, 100))
            user01_template = cv2.equalizeHist(user01_template)
            user01_hist = cv2.calcHist([user01_template], [0], None, [256], [0, 256])
            cv2.normalize(user01_hist, user01_hist, 0, 1, cv2.NORM_MINMAX)
            user01_loaded = True

reload_user_template()

ATTENDANCE_FILE = os.path.join(BASE_DIR, 'attendance.csv')
if not os.path.exists(ATTENDANCE_FILE):
    with open(ATTENDANCE_FILE, 'w', encoding='utf-8') as f:
        f.write("Date,Time,Name,Score,Camera,SnapshotPath\n")

attendance_log = []
last_user01_photo_time = 0
last_registered_banner_time = 0

def sync_attendance_csv():
    try:
        with open(ATTENDANCE_FILE, 'w', encoding='utf-8') as f:
            f.write("Date,Time,Name,Score,Camera,SnapshotPath\n")
            for item in reversed(attendance_log):
                f.write(f"{item['Date']},{item['Time']},{item['Name']},{item.get('Score','-')},{item['Camera']},{item['Path']}\n")
    except Exception:
        pass

def compare_face(face_gray_roi):
    if not user01_loaded or user01_template is None:
        return "UNKNOWN", 0.0
    if face_gray_roi is None or face_gray_roi.size == 0:
        return "IGNORE", 0.0
    try:
        face_resized = cv2.resize(face_gray_roi, (100, 100))
        face_eq = cv2.equalizeHist(face_resized)
        res = cv2.matchTemplate(face_eq, user01_template, cv2.TM_CCOEFF_NORMED)
        tm_score = float(res[0][0])
        hist = cv2.calcHist([face_eq], [0], None, [256], [0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        hist_score = float(cv2.compareHist(user01_hist, hist, cv2.HISTCMP_CORREL))
        combined_score = max(0.0, (tm_score * 0.7) + (hist_score * 0.3))

        if combined_score >= face_threshold:
            return "USER01", combined_score
    except Exception:
        pass
    return "IGNORE", 0.0

latest_csi_raw_frame = None
csi_frame_lock = threading.Lock()

def draw_timestamp_overlay(frame):
    now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    h, w, _ = frame.shape
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 2
    (tw, th), _ = cv2.getTextSize(now_str, font, scale, thickness)
    
    x = w - tw - 15
    y = th + 15
    cv2.rectangle(frame, (x - 8, y - th - 6), (w - 7, y + 6), (0, 0, 0), -1)
    cv2.putText(frame, now_str, (x, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

def process_face_recognition(bgr_frame, gray_eq_frame):
    global last_user01_photo_time, last_registered_banner_time
    if face_cascade.empty() or gray_eq_frame is None or gray_eq_frame.size == 0:
        return
    try:
        faces = face_cascade.detectMultiScale(gray_eq_frame, scaleFactor=1.1, minNeighbors=4, minSize=(40, 40))
    except Exception:
        return

    cur_time = time.time()
    for (x, y, w, h) in faces:
        name, score = compare_face(gray_eq_frame[y:y+h, x:x+w])
        if name == "USER01":
            color = (0, 255, 0)
            cv2.rectangle(bgr_frame, (x, y), (x+w, y+h), color, 2)
            cv2.putText(bgr_frame, f"USER01 ({score:.2f})", (x, max(25, y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if cur_time - last_user01_photo_time >= 5.0:
                last_user01_photo_time = cur_time
                last_registered_banner_time = cur_time
                trigger_snapshot_buzzer()

                now = datetime.datetime.now()
                date_str = now.strftime("%Y-%m-%d")
                time_str = now.strftime("%H:%M:%S")
                timestamp_file = now.strftime("%Y%m%d_%H%M%S")
                snap_filename = f"{timestamp_file}_USER01.jpg"
                snap_path = os.path.join(SNAPSHOTS_DIR, snap_filename)
                
                try:
                    save_frame = bgr_frame.copy()
                    draw_timestamp_overlay(save_frame)
                    cv2.imwrite(snap_path, save_frame)

                    log_entry = {
                        "Date": date_str,
                        "Time": time_str,
                        "Name": "USER01",
                        "Score": f"{score:.2f}",
                        "Camera": "前方CSIカメラ",
                        "Path": snap_filename
                    }
                    attendance_log.insert(0, log_entry)
                    sync_attendance_csv()
                except Exception:
                    pass

def draw_attendance_banner(frame):
    if time.time() - last_registered_banner_time < 3.0:
        h, w, _ = frame.shape
        cv2.rectangle(frame, (0, h - 50), (w, h), (0, 180, 0), -1)
        cv2.putText(frame, "RECOGNIZED: USER01 (PHOTO SAVED)", (int(w * 0.08), h - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)

# --- ArUco 検出 ---
def get_aruco_dict(name):
    if hasattr(cv2.aruco, name):
        val = getattr(cv2.aruco, name)
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            return cv2.aruco.getPredefinedDictionary(val)
        elif hasattr(cv2.aruco, 'Dictionary_get'):
            return cv2.aruco.Dictionary_get(val)
    return None

dict_names = ['DICT_4X4_50', 'DICT_4X4_100', 'DICT_5X5_50', 'DICT_5X5_100', 'DICT_6X6_50', 'DICT_ARUCO_ORIGINAL']
ARUCO_DICTS = [d for d in [get_aruco_dict(n) for n in dict_names] if d is not None]

if hasattr(cv2.aruco, 'DetectorParameters_create'):
    aruco_params = cv2.aruco.DetectorParameters_create()
elif hasattr(cv2.aruco, 'DetectorParameters'):
    aruco_params = cv2.aruco.DetectorParameters()
else:
    aruco_params = None

if aruco_params is not None:
    aruco_params.adaptiveThreshWinSizeMin = 3
    aruco_params.adaptiveThreshWinSizeMax = 23
    aruco_params.adaptiveThreshWinSizeStep = 10
    aruco_params.adaptiveThreshConstant = 7
    aruco_params.minMarkerPerimeterRate = 0.10
    aruco_params.maxMarkerPerimeterRate = 4.0
    aruco_params.polygonalApproxAccuracyRate = 0.03
    aruco_params.minCornerDistanceRate = 0.05
    aruco_params.minDistanceToBorder = 3
    aruco_params.maxErroneousBitsInBorderRate = 0.35
    if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

def detect_any_aruco(img):
    if img is None or img.size == 0:
        return None, None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

    for d in ARUCO_DICTS:
        try:
            if hasattr(cv2.aruco, 'ArucoDetector') and hasattr(cv2.aruco, 'DetectorParameters'):
                det = cv2.aruco.ArucoDetector(d, aruco_params)
                corners, ids, _ = det.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=aruco_params)
            if ids is not None and len(ids) > 0:
                return corners, ids
        except Exception:
            pass
    return None, None

# --- 前方CSIカメラ初期化 ---
try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()
    csi_available = True
except Exception:
    csi_available = False

def find_working_usb_camera():
    dev_paths = sorted(glob.glob('/dev/video*'), key=lambda x: int(x.replace('/dev/video', '')) if x.replace('/dev/video', '').isdigit() else 999)
    for dev_path in dev_paths:
        try:
            idx = int(dev_path.replace('/dev/video', ''))
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                for _ in range(3):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        return cap, idx
                    time.sleep(0.01)
                cap.release()
        except Exception:
            pass
    return None, None

# --- 甲板USBカメラ バックグラウンド常時キャプチャ ---
latest_deck_frame_jpeg = None
deck_jpeg_lock = threading.Lock()

align_state = {
    "target_found": False,
    "dx": 0,
    "dy": 0,
    "angle": 0.0,
    "pos_ok": False,
    "angle_ok": False,
    "last_seen_time": 0
}
align_state_lock = threading.Lock()

def deck_camera_capture_loop():
    global latest_deck_frame_jpeg
    cap = None
    DEAD_ZONE_POS = 30
    DEAD_ZONE_ANGLE = 8

    while True:
        if cap is None or not cap.isOpened():
            cap, _ = find_working_usb_camera()
            if cap is None:
                blank = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(blank, "SEARCHING USB CAMERA...", (140, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                _, buffer = cv2.imencode('.jpg', blank)
                with deck_jpeg_lock:
                    latest_deck_frame_jpeg = buffer.tobytes()
                time.sleep(1.0)
                continue

        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            cap.release()
            cap = None
            time.sleep(0.1)
            continue

        h, w, _ = frame.shape
        center_x = (w // 2) + DECK_OFFSET_X
        center_y = (h // 2) + DECK_OFFSET_Y

        corners, ids = detect_any_aruco(frame)
        display_frame = cv2.flip(frame, 1)

        target_found = False
        dx, dy, marker_angle_deg = 0, 0, 0.0
        pos_ok, angle_ok = False, False

        if ids is not None and len(ids) > 0:
            c = corners[0][0]
            c_disp = np.array([
                [w - 1 - c[0][0], c[0][1]],
                [w - 1 - c[1][0], c[1][1]],
                [w - 1 - c[2][0], c[2][1]],
                [w - 1 - c[3][0], c[3][1]]
            ], dtype=np.int32)
            
            cv2.polylines(display_frame, [c_disp], True, (0, 255, 0), 2)
            cx = int((c_disp[0][0] + c_disp[1][0] + c_disp[2][0] + c_disp[3][0]) / 4)
            cy = int((c_disp[0][1] + c_disp[1][1] + c_disp[2][1] + c_disp[3][1]) / 4)
            
            cv2.line(display_frame, (cx, cy), (center_x, center_y), (0, 255, 255), 2)
            cv2.circle(display_frame, (cx, cy), 6, (0, 0, 255), -1)

            top_mid_x = (c_disp[0][0] + c_disp[1][0]) / 2.0
            top_mid_y = (c_disp[0][1] + c_disp[1][1]) / 2.0
            marker_angle_deg = math.degrees(math.atan2(top_mid_x - cx, -(top_mid_y - cy)))
            
            dx = cx - center_x
            dy = cy - center_y

            pos_ok = (abs(dx) <= DEAD_ZONE_POS and abs(dy) <= DEAD_ZONE_POS)
            angle_ok = (abs(marker_angle_deg) <= DEAD_ZONE_ANGLE)
            target_found = True

            guide = "ACTION: [OK] READY TO LAND" if (pos_ok and angle_ok) else "ACTION: ADJUSTING POSITION"
            cv2.putText(display_frame, f"ID:{ids[0][0]} | dX:{dx:+d} dY:{dy:+d}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(display_frame, guide, (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0) if (pos_ok and angle_ok) else (255, 255, 255), 2)

            with align_state_lock:
                align_state["target_found"] = True
                align_state["dx"] = dx
                align_state["dy"] = dy
                align_state["angle"] = marker_angle_deg
                align_state["pos_ok"] = pos_ok
                align_state["angle_ok"] = angle_ok
                align_state["last_seen_time"] = time.time()
        else:
            with align_state_lock:
                if time.time() - align_state["last_seen_time"] > 0.25:
                    align_state["target_found"] = False

            cv2.putText(display_frame, "SEARCHING ARUCO MARKER...", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.drawMarker(display_frame, (center_x, center_y), (0, 255, 0), cv2.MARKER_CROSS, 40, 2)
        cv2.circle(display_frame, (center_x, center_y), 30, (0, 255, 0), 1)

        _, buffer = cv2.imencode('.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with deck_jpeg_lock:
            latest_deck_frame_jpeg = buffer.tobytes()
        time.sleep(0.03)

threading.Thread(target=deck_camera_capture_loop, daemon=True).start()

# --- Tello ドローン連携 ---
tello_drone = None
tello_connected = False
tello_battery = "--"
latest_tello_frame_jpeg = None
latest_tello_raw_frame = None
tello_jpeg_lock = threading.Lock()
tello_frame_lock = threading.Lock()
tello_rc_lock = threading.Lock()

def tello_background_loop():
    global tello_drone, tello_connected, tello_battery, latest_tello_frame_jpeg, latest_tello_raw_frame
    if not HAS_TELLO_LIB:
        return

    while True:
        if tello_drone is None:
            try:
                t = Tello()
                t.connect()
                t.streamon()
                tello_drone = t
                tello_connected = True
            except Exception:
                tello_drone = None
                tello_connected = False
                time.sleep(2.0)
                continue

        try:
            try:
                tello_battery = tello_drone.get_battery()
            except Exception:
                pass

            frame_read = tello_drone.get_frame_read()
            if frame_read is not None and frame_read.frame is not None:
                t_frame = frame_read.frame.copy()
                t_frame = cv2.cvtColor(t_frame, cv2.COLOR_RGB2BGR)
                
                with tello_frame_lock:
                    latest_tello_raw_frame = t_frame.copy()

                # Telloカメラ映像の左上にバッテリー、右上に現在日時を描画
                cv2.putText(t_frame, f"TELLO BAT: {tello_battery}%", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                draw_timestamp_overlay(t_frame)

                _, buffer = cv2.imencode('.jpg', t_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                with tello_jpeg_lock:
                    latest_tello_frame_jpeg = buffer.tobytes()
            time.sleep(0.04)
        except Exception:
            tello_connected = False
            tello_drone = None
            time.sleep(1.0)

threading.Thread(target=tello_background_loop, daemon=True).start()

def generate_tello_frames():
    while True:
        with tello_jpeg_lock:
            frame_bytes = latest_tello_frame_jpeg
        if frame_bytes is not None:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            blank = np.zeros((360, 480, 3), np.uint8)
            cv2.putText(blank, "TELLO DISCONNECTED", (90, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            _, buffer = cv2.imencode('.jpg', blank)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.05)

# --- Pico シリアル通信 ---
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
    ser.reset_input_buffer()
except Exception:
    ser = None

def send_pico_command(cmd_str):
    if ser:
        try:
            ser.write((cmd_str + "\n").encode('utf-8'))
            ser.flush()
        except Exception:
            pass

def set_pico_yellow_led(state: bool):
    val = "1" if state else "0"
    send_pico_command(f"LED_YELLOW:{val}")
    send_pico_command(f"YELLOW:{val}")
    send_pico_command(f"LED_GREEN:{val}")

# --- 状態変数 ---
water_detected = False
last_water_detected_time = 0
WATER_TIMEOUT_SEC = 5.0
is_aligning = False
current_action_status = "☀️ 陸上待機中 (操縦可能)"

# --- 着陸アライメント自律補正ループ ---
def alignment_worker():
    global is_aligning, current_action_status
    DEAD_ZONE_POS = 30
    DEAD_ZONE_ANGLE = 8

    dist_history = deque(maxlen=6)
    boost_level = 0
    phase_start_time = time.time()
    min_dist_in_phase = float('inf')

    while True:
        if is_aligning and not water_detected:
            current_action_status = "🎯 着陸アライメント中：ドローン直下に位置・角度調整中"
            dist_history.clear()
            boost_level = 0
            phase_start_time = time.time()
            min_dist_in_phase = float('inf')

            while is_aligning and not water_detected:
                with align_state_lock:
                    target_found = align_state["target_found"]
                    dx = align_state["dx"]
                    dy = align_state["dy"]
                    marker_angle_deg = align_state["angle"]
                    pos_ok = align_state["pos_ok"]
                    angle_ok = align_state["angle_ok"]

                if target_found:
                    current_dist = math.hypot(dx, dy)
                    dist_history.append(current_dist)
                    ma_dist = sum(dist_history) / len(dist_history)

                    now = time.time()
                    if ma_dist < min_dist_in_phase - 5.0:
                        min_dist_in_phase = ma_dist
                        phase_start_time = now
                        if boost_level != 0:
                            boost_level = 0
                    
                    elapsed = now - phase_start_time
                    if boost_level == 0:
                        if elapsed >= 1.0:
                            boost_level = 1
                            phase_start_time = now
                            min_dist_in_phase = ma_dist
                    elif boost_level == 1:
                        if elapsed >= 1.0:
                            boost_level = 2
                            phase_start_time = now
                            min_dist_in_phase = ma_dist
                    elif boost_level == 2:
                        if elapsed >= 1.0:
                            boost_level = 0
                            phase_start_time = now
                            min_dist_in_phase = ma_dist

                    lateral_speed = 650 if boost_level == 0 else (700 if boost_level == 1 else 750)
                    linear_speed = 600

                    if pos_ok and angle_ok:
                        set_car_move('stop')
                        current_action_status = "✅ 【位置照準完了】ドローン着陸待機OK（Tello着陸ボタンを押してください）"
                        boost_level = 0
                    else:
                        if not pos_ok:
                            if abs(dx) <= DEAD_ZONE_POS:
                                current_action_status = f"🎯 前後微調整中 (速度: {linear_speed} 固定)"
                                set_car_move('forward' if dy < -DEAD_ZONE_POS else 'backward', linear_speed)
                                if abs(dy) < 45:
                                    time.sleep(0.04)
                                    set_car_move('stop')
                            elif abs(dy) <= DEAD_ZONE_POS:
                                current_action_status = f"🎯 横スライド補正中 (速度: {lateral_speed})"
                                set_car_move('slide_right' if dx > DEAD_ZONE_POS else 'slide_left', lateral_speed)
                                if abs(dx) < 45:
                                    time.sleep(0.04)
                                    set_car_move('stop')
                            else:
                                current_action_status = f"🎯 斜めスライド追従中 (速度: {lateral_speed})"
                                if dy < -DEAD_ZONE_POS and dx < -DEAD_ZONE_POS:
                                    set_car_move('slide_forward_left', lateral_speed)
                                elif dy < -DEAD_ZONE_POS and dx > DEAD_ZONE_POS:
                                    set_car_move('slide_forward_right', lateral_speed)
                                elif dy > DEAD_ZONE_POS and dx < -DEAD_ZONE_POS:
                                    set_car_move('slide_backward_left', lateral_speed)
                                elif dy > DEAD_ZONE_POS and dx > DEAD_ZONE_POS:
                                    set_car_move('slide_backward_right', lateral_speed)
                        else:
                            current_action_status = f"🎯 向き（角度）一致補正中 (速度: {lateral_speed})"
                            set_car_move('turn_right' if marker_angle_deg > DEAD_ZONE_ANGLE else 'turn_left', lateral_speed)
                            time.sleep(0.04)
                            set_car_move('stop')
                else:
                    set_car_move('stop')
                    current_action_status = "🔍 ArUcoマーカー探索中..."
                    dist_history.clear()
                    min_dist_in_phase = float('inf')
                    phase_start_time = time.time()
                    boost_level = 0

                time.sleep(0.08)

            set_car_move('stop')
        time.sleep(0.2)

threading.Thread(target=alignment_worker, daemon=True).start()

# --- Pico 水センサー監視 ---
def monitor_pico_sensor():
    global water_detected, last_water_detected_time, current_action_status, is_aligning
    while True:
        if ser:
            try:
                latest_line = ""
                while ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        latest_line = line

                if "WET:1" in latest_line:
                    last_water_detected_time = time.time()
                    if not water_detected:
                        water_detected = True
                        if is_aligning:
                            is_aligning = False
                            set_pico_yellow_led(False)
                        set_car_move('stop')
                    set_screw_motors(70)
                    current_action_status = "🌊【水上モード】青色LED点灯＆スクリュー推進中"
            except Exception:
                pass

        if water_detected:
            elapsed = time.time() - last_water_detected_time
            if elapsed >= WATER_TIMEOUT_SEC:
                water_detected = False
                set_screw_motors(0)
                set_car_move('stop')
                current_action_status = "☀️【陸上復帰】水未検知 ➔ 待機中"
        time.sleep(0.05)

threading.Thread(target=monitor_pico_sensor, daemon=True).start()

# --- 前方CSIカメラストリーム配信 ---
def generate_csi_frames():
    global latest_csi_raw_frame
    while True:
        if csi_available and picam2:
            try:
                raw = picam2.capture_array()
                if raw is None or raw.size == 0:
                    time.sleep(0.02)
                    continue

                if len(raw.shape) == 3:
                    bgr = cv2.cvtColor(raw, cv2.COLOR_RGBA2BGR if raw.shape[2] == 4 else cv2.COLOR_RGB2BGR)
                    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                else:
                    gray = raw
                    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

                with csi_frame_lock:
                    latest_csi_raw_frame = bgr.copy()

                gray_eq = cv2.equalizeHist(gray)
                process_face_recognition(bgr, gray_eq)
                draw_attendance_banner(bgr)
                draw_timestamp_overlay(bgr)

                _, buffer = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                time.sleep(0.01)
            except Exception:
                time.sleep(0.05)
        else:
            time.sleep(0.1)

def generate_deck_frames():
    while True:
        with deck_jpeg_lock:
            frame_bytes = latest_deck_frame_jpeg
        if frame_bytes is not None:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.04)

# --- Flask ルーティング ---
template_dirs = [os.path.join(BASE_DIR, 'templates'), BASE_DIR]
app = Flask(__name__, template_folder=template_dirs[0] if os.path.exists(template_dirs[0]) else template_dirs[1])

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/snapshots/<filename>')
def get_snapshot(filename):
    return send_from_directory(SNAPSHOTS_DIR, filename)

@app.route('/video_feed/front')
def video_feed_front():
    return Response(generate_csi_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/deck')
def video_feed_deck():
    return Response(generate_deck_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/tello')
def video_feed_tello():
    return Response(generate_tello_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    d = get_safe_distance()
    bat = get_car_battery_percentage()
    return jsonify({
        "raspi_online": True,
        "car_battery": bat,
        "battery_critical": (bat <= 10),
        "water_detected": water_detected,
        "is_aligning": is_aligning,
        "current_speed": current_speed,
        "face_threshold": face_threshold,
        "brake_distance_cm": brake_distance_cm,
        "front_distance": round(d, 1) if d < 400 else 999.0,
        "forward_locked": (d <= brake_distance_cm),
        "action_status": current_action_status,
        "attendance_log": attendance_log,
        "tello_connected": tello_connected,
        "tello_battery": str(tello_battery)
    })

# --- 速度設定 API (700〜1600, 100刻み) ---
@app.route('/api/car/speed', methods=['POST'])
def api_set_speed():
    global current_speed, brake_distance_cm
    data = request.get_json(force=True)
    speed = int(data.get('speed', 800))
    current_speed = max(700, min(1600, speed))
    
    if current_speed >= 1200:
        brake_distance_cm = 50.0
    else:
        brake_distance_cm = user_custom_brake_dist

    return jsonify({
        "status": "ok",
        "speed": current_speed,
        "brake_distance_cm": brake_distance_cm
    })

@app.route('/api/face/threshold', methods=['POST'])
def api_set_face_threshold():
    global face_threshold
    data = request.get_json(force=True)
    val = float(data.get('threshold', 0.50))
    face_threshold = max(0.20, min(0.80, val))
    return jsonify({"status": "ok", "threshold": face_threshold})

@app.route('/api/car/brake_dist', methods=['POST'])
def api_set_brake_dist():
    global brake_distance_cm, user_custom_brake_dist
    data = request.get_json(force=True)
    val = float(data.get('distance', 25.0))
    user_custom_brake_dist = max(10.0, min(60.0, val))
    
    if current_speed < 1200:
        brake_distance_cm = user_custom_brake_dist
    return jsonify({"status": "ok", "brake_distance_cm": brake_distance_cm})

@app.route('/api/car/manual', methods=['POST'])
def api_car_manual():
    global current_moving_direction, current_action_status, is_aligning
    if water_detected:
        return jsonify({"status": "error", "reason": "water_detected"})

    data = request.get_json(force=True)
    direction = data.get('direction', 'stop')

    d = get_safe_distance()
    if d <= brake_distance_cm and direction in ['forward', 'slide_forward_left', 'slide_forward_right']:
        set_car_move('stop')
        current_moving_direction = 'stop'
        current_action_status = f"🚫 前方障害物検知 ({d:.1f}cm <= {brake_distance_cm:.0f}cm)：前進不可！他の方向に移動してください"
        return jsonify({"status": "blocked", "reason": "obstacle_ahead", "distance": d})

    if is_aligning and direction != 'stop':
        is_aligning = False
        set_pico_yellow_led(False)

    current_moving_direction = direction
    set_car_move(direction, current_speed)
    
    if direction != 'stop':
        current_action_status = f"🎮 手動操縦中: {direction.upper()} (速度: {current_speed})"
    else:
        current_action_status = "手動停止 (待機中)"
    return jsonify({"status": "ok", "direction": direction})

# --- Tello 操縦 API ---
@app.route('/api/tello/action', methods=['POST'])
def api_tello_action():
    if not tello_drone or not tello_connected:
        return jsonify({"status": "error", "message": "Tello not connected"})
    
    data = request.get_json(force=True)
    act = data.get('action')
    try:
        if act == 'takeoff':
            threading.Thread(target=tello_drone.takeoff, daemon=True).start()
        elif act == 'land':
            threading.Thread(target=tello_drone.land, daemon=True).start()
        elif act == 'emergency':
            tello_drone.emergency()
        return jsonify({"status": "ok", "action": act})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/tello/rc', methods=['POST'])
def api_tello_rc():
    if not tello_drone or not tello_connected:
        return jsonify({"status": "error", "message": "Tello not connected"})
    
    data = request.get_json(force=True)
    lr = int(data.get('lr', 0))
    fb = int(data.get('fb', 0))
    ud = int(data.get('ud', 0))
    yaw = int(data.get('yaw', 0))
    
    try:
        with tello_rc_lock:
            tello_drone.send_rc_control(lr, fb, ud, yaw)
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

# --- Tello 手動写真撮影 API ---
@app.route('/api/snapshot/tello', methods=['POST'])
def api_snapshot_tello():
    trigger_snapshot_buzzer()
    
    frame = None
    with tello_frame_lock:
        if latest_tello_raw_frame is not None:
            frame = latest_tello_raw_frame.copy()

    if frame is not None:
        now = datetime.datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S")
        timestamp_file = now.strftime("%Y%m%d_%H%M%S")
        snap_filename = f"{timestamp_file}_TELLO_MANUAL.jpg"
        snap_path = os.path.join(SNAPSHOTS_DIR, snap_filename)
        
        try:
            save_frame = frame.copy()
            draw_timestamp_overlay(save_frame)
            cv2.imwrite(snap_path, save_frame)

            log_entry = {
                "Date": date_str,
                "Time": time_str,
                "Name": "MANUAL",
                "Score": "-",
                "Camera": "🛸 Telloカメラ",
                "Path": snap_filename
            }
            attendance_log.insert(0, log_entry)
            sync_attendance_csv()
            return jsonify({"status": "ok", "path": snap_filename})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)})
    return jsonify({"status": "error", "message": "no_frame"})

# --- 前方カメラ 任意写真撮影 API ---
@app.route('/api/snapshot/manual', methods=['POST'])
def api_snapshot_manual():
    trigger_snapshot_buzzer()
    
    frame = None
    with csi_frame_lock:
        if latest_csi_raw_frame is not None:
            frame = latest_csi_raw_frame.copy()

    if frame is not None:
        now = datetime.datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S")
        timestamp_file = now.strftime("%Y%m%d_%H%M%S")
        snap_filename = f"{timestamp_file}_MANUAL.jpg"
        snap_path = os.path.join(SNAPSHOTS_DIR, snap_filename)
        
        try:
            save_frame = frame.copy()
            draw_timestamp_overlay(save_frame)
            cv2.imwrite(snap_path, save_frame)

            log_entry = {
                "Date": date_str,
                "Time": time_str,
                "Name": "MANUAL",
                "Score": "-",
                "Camera": "前方CSIカメラ",
                "Path": snap_filename
            }
            attendance_log.insert(0, log_entry)
            sync_attendance_csv()
            return jsonify({"status": "ok", "path": snap_filename})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)})
    return jsonify({"status": "error", "message": "no_frame"})

@app.route('/api/alignment/start', methods=['POST'])
def api_alignment_start():
    global is_aligning, current_action_status
    if water_detected:
        return jsonify({"status": "error", "reason": "water_detected"})
    is_aligning = True
    set_pico_yellow_led(True)
    current_action_status = "🎯 着陸アライメント開始 ➔ 【黄色LED点灯】"
    return jsonify({"status": "alignment_started"})

@app.route('/api/alignment/stop', methods=['POST'])
def api_alignment_stop():
    global is_aligning, current_action_status
    is_aligning = False
    set_pico_yellow_led(False)
    set_car_move('stop')
    current_action_status = "着陸アライメント終了 (待機中)"
    return jsonify({"status": "alignment_stopped"})

@app.route('/api/attendance/delete/<path_filename>', methods=['POST', 'DELETE'])
def delete_attendance_entry(path_filename):
    global attendance_log
    initial_count = len(attendance_log)
    attendance_log = [item for item in attendance_log if item['Path'] != path_filename]
    sync_attendance_csv()
    snap_full_path = os.path.join(SNAPSHOTS_DIR, path_filename)
    if os.path.exists(snap_full_path):
        try:
            os.remove(snap_full_path)
        except Exception:
            pass
    return jsonify({"status": "deleted", "target": path_filename, "success": initial_count > len(attendance_log)})

@app.route('/api/raspi/shutdown', methods=['POST'])
def shutdown_raspi():
    set_car_move('stop')
    set_screw_motors(0)
    set_pico_yellow_led(False)
    def _shutdown_delay():
        time.sleep(1.0)
        os.system('sudo shutdown -h now')
    threading.Thread(target=_shutdown_delay, daemon=True).start()
    return jsonify({"status": "shutdown_initiated"})

if __name__ == '__main__':
    print("\n==================================================")
    print("  体験型エンタメ ～海陸両用空母ロボット～ 統合サーバー")
    print("  速度700-1600(100刻み)・Tello撮影ボタン＆UI整理版")
    print("==================================================\n")
    app.run(host='0.0.0.0', port=5000, debug=False)