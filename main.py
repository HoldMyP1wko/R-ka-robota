import os
import cv2
import mediapipe as mp
import numpy as np
import socket
import time
import signal
import sys
os.environ["DISPLAY"] = ":0"

# =============================================================
#  FLAGA DEBUGOWANIA
#  Zmień na False, gdy wgrywasz na Raspberry Pi na wystawkę!
# =============================================================
DEBUG = False
# =============================================================

# =============================================================
#  CZUŁOŚĆ I FILTRY
# =============================================================
FINGER_ALPHA = 0.15
FINGER_DEADBAND = 1.5

WRIST_ALPHA = 0.30
WRIST_DEADBAND = 0.05

RETURN_ALPHA = 0.05
TIMEOUT_SECONDS = 3.0  # Czas po jakim wraca do pięści przy braku PRAWEJ dłoni
# =============================================================

# --- KONFIGURACJA UDP ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

os.environ["GLOG_minloglevel"] = "2"

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

FINGERS_INDEXES = {
    "Thumb": (1, 2, 4),
    "Index": (5, 6, 8),
    "Middle": (9, 10, 12),
    "Ring": (13, 14, 16),
    "Pinky": (17, 18, 20),
}

# =============================================================
#  MAPOWANIE SERWO
# =============================================================
SERVO_MAP = {
    "Thumb": {
        "open_pwm": 100, "close_pwm": 500,
        "input_min": 110, "input_max": 177
    },
    "Index": {
        "open_pwm": 100, "close_pwm": 440,
        "input_min": 15, "input_max": 177
    },
    "Middle": {
        "open_pwm": 525, "close_pwm": 100,  # Odwrócone serwo - program sam to przeliczy!
        "input_min": 16, "input_max": 179
    },
    "Ring": {
        "open_pwm": 100, "close_pwm": 430,
        "input_min": 18, "input_max": 176
    },
    "Pinky": {
        "open_pwm": 100, "close_pwm": 390,
        "input_min": 33, "input_max": 171
    },
    "Wrist": {
        "open_pwm": 110, "close_pwm": 600,
        "input_min": 0.10, "input_max": 0.85  # ZMIENIONE: Teraz pokrywa się z Twoją ręką!
    }
}

ORDER = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Wrist"]


# =============================================================
#  FUNKCJE POMOCNICZE (PIĘŚĆ I DŁOŃ)
# =============================================================
def get_fist_pwm():
    vals = []
    for name in ORDER:
        if name == "Wrist":
            vals.append(SERVO_MAP[name]["open_pwm"])
        else:
            vals.append(SERVO_MAP[name]["close_pwm"])
    return vals


def get_open_pwm():
    return [SERVO_MAP[name]["open_pwm"] for name in ORDER]


# =============================================================
#  BEZPIECZNE ZAMYKANIE (SYSTEMD / SIGTERM)
# =============================================================
def bezpieczne_zamkniecie(signum, frame):
    if DEBUG:
        print("\n[System] Otrzymano sygnał SIGTERM. Zamykam skrypt...")
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, bezpieczne_zamkniecie)


# =============================================================
#  KALIBRACJA - EFEKT FALI
# =============================================================
def run_calibration_wave(sock):
    if DEBUG: print("\n[System] Rozpoczynam kalibrację (fala)...")

    # Startujemy z w pełni otwartej dłoni (Dioda ZAPALONA = 1)
    current_vals = get_open_pwm()
    try:
        sock.sendto((",".join(map(str, current_vals)) + ",1").encode(), (UDP_IP, UDP_PORT))
    except Exception:
        pass
    time.sleep(0.3)

    wave_order = ["Pinky", "Ring", "Middle", "Index", "Thumb"]

    for finger in wave_order:
        idx = ORDER.index(finger)
        current_vals[idx] = SERVO_MAP[finger]["close_pwm"]
        try:
            sock.sendto((",".join(map(str, current_vals)) + ",1").encode(), (UDP_IP, UDP_PORT))
        except Exception:
            pass
        time.sleep(0.4)

    time.sleep(0.3)

    # Pełne otwarcie na koniec fali (Dioda ZAPALONA = 1)
    open_vals = get_open_pwm()
    try:
        sock.sendto((",".join(map(str, open_vals)) + ",1").encode(), (UDP_IP, UDP_PORT))
    except Exception:
        pass
    time.sleep(0.6)

    if DEBUG: print("[System] Kalibracja zakończona.\n")


# =============================================================
#  KLASY I OBLICZENIA MATEMATYCZNE
# =============================================================
class AngleSmoother:
    def __init__(self, alpha=0.4, deadband=2.0):
        self.alpha = float(alpha)
        self.dead = float(deadband)
        self.ema = None
        self.last_target = None

    def update(self, new_val):
        new_val = float(new_val)
        if self.ema is None:
            self.ema = new_val
            self.last_target = new_val
            return new_val

        if abs(new_val - self.last_target) > self.dead:
            self.last_target = new_val

        self.ema = (1.0 - self.alpha) * self.ema + self.alpha * self.last_target
        return self.ema


class ReturnSmoother:
    def __init__(self, alpha=0.05):
        self.alpha = float(alpha)
        self.current = {}

    def init_value(self, name, default_val):
        if name not in self.current:
            self.current[name] = float(default_val)

    def update_toward_default(self, name, default_val):
        target = float(default_val)
        self.current[name] = (1.0 - self.alpha) * self.current[name] + self.alpha * target
        return self.current[name]

    def set_value(self, name, val):
        self.current[name] = float(val)


def get_hand_rotation_angle(lm):
    wrist = np.array([lm[0].x, lm[0].y])
    middle = np.array([lm[9].x, lm[9].y])
    axis = middle - wrist
    angle = np.degrees(np.arctan2(axis[0], -axis[1]))
    return angle


def normalize_landmarks(lm, rotation_angle_deg):
    angle_rad = np.radians(rotation_angle_deg)
    cos_a = np.cos(-angle_rad)
    sin_a = np.sin(-angle_rad)
    rot = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
    origin = np.array([lm[0].x, lm[0].y])
    normalized = []
    for point in lm:
        p = np.array([point.x, point.y]) - origin
        p_rot = rot @ p
        normalized.append(p_rot)
    return normalized


def calculate_finger_angle_from_normalized(norm_lm, p1, p2, p3):
    a, b, c = norm_lm[p1], norm_lm[p2], norm_lm[p3]
    ba, bc = a - b, c - b
    norm_ba, norm_bc = np.linalg.norm(ba), np.linalg.norm(bc)
    if norm_ba == 0 or norm_bc == 0: return 0.0
    cosine = np.dot(ba, bc) / (norm_ba * norm_bc)
    return np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))


def get_wrist_pronation_3d(lm, is_right_hand=True):
    ux, uy, uz = lm[5].x - lm[0].x, lm[5].y - lm[0].y, lm[5].z - lm[0].z
    vx, vy, vz = lm[17].x - lm[0].x, lm[17].y - lm[0].y, lm[17].z - lm[0].z
    nx, nz = uy * vz - uz * vy, uz * vy - uy * vx
    if not is_right_hand: nx, nz = -nx, -nz
    angle_rad = np.arctan2(nx, -nz)
    return float(np.clip(angle_rad / np.pi, -1.0, 1.0))


def map_smart(value, cfg):
    in_min, in_max = cfg["input_min"], cfg["input_max"]
    pwm_open, pwm_close = cfg["open_pwm"], cfg["close_pwm"]
    val = max(in_min, min(value, in_max))
    norm = (val - in_min) / (in_max - in_min)
    return int(pwm_close + norm * (pwm_open - pwm_close))


# =============================================================
#  FUNKCJE DEBUGOWANIA
# =============================================================
def draw_hand_axis(frame, lm, w, h):
    x0, y0 = int(lm[0].x * w), int(lm[0].y * h)
    x9, y9 = int(lm[9].x * w), int(lm[9].y * h)
    cv2.line(frame, (x0, y0), (x9, y9), (255, 100, 0), 3)
    cv2.circle(frame, (x0, y0), 6, (0, 255, 255), -1)
    cv2.circle(frame, (x9, y9), 6, (255, 100, 0), -1)


def draw_wrist_outline(frame, lm, w, h):
    if not hasattr(draw_wrist_outline, "prev_p1"):
        draw_wrist_outline.prev_p1 = None
        draw_wrist_outline.prev_p2 = None

    x0, y0 = int(lm[0].x * w), int(lm[0].y * h)
    x9, y9 = int(lm[9].x * w), int(lm[9].y * h)

    dx = x9 - x0
    dy = y9 - y0
    length = np.sqrt(dx ** 2 + dy ** 2) + 0.001
    dx /= length
    dy /= length
    px = -dy
    py = dx
    wrist_width_half = int(length * 0.35)

    p1_raw = np.array([x0 + px * wrist_width_half, y0 + py * wrist_width_half])
    p2_raw = np.array([x0 - px * wrist_width_half, y0 - py * wrist_width_half])

    alpha = 0.8

    if draw_wrist_outline.prev_p1 is None:
        draw_wrist_outline.prev_p1 = p1_raw
        draw_wrist_outline.prev_p2 = p2_raw
    else:
        draw_wrist_outline.prev_p1 = (1 - alpha) * draw_wrist_outline.prev_p1 + alpha * p1_raw
        draw_wrist_outline.prev_p2 = (1 - alpha) * draw_wrist_outline.prev_p2 + alpha * p2_raw

    p1 = (int(draw_wrist_outline.prev_p1[0]), int(draw_wrist_outline.prev_p1[1]))
    p2 = (int(draw_wrist_outline.prev_p2[0]), int(draw_wrist_outline.prev_p2[1]))

    cv2.line(frame, p1, p2, (0, 255, 200), 4)
    cv2.circle(frame, p1, 5, (0, 200, 255), -1)
    cv2.circle(frame, p2, 5, (0, 200, 255), -1)
    cv2.putText(frame, "WRIST", (p2[0] - 20, p2[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 2)


def draw_pwm_panel(frame, pwm_values, wrist_angle):
    panel_x, panel_y, line_h = 10, 10, 26
    font = cv2.FONT_HERSHEY_SIMPLEX
    # Zwiększona wysokość panelu, żeby zmieścić informację o diodzie
    panel_w, panel_h = 270, line_h * (len(ORDER) + 4) + 10
    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, "=== UDP OUTPUT ===", (panel_x + 5, panel_y + line_h), font, 0.5, (0, 255, 255), 1)

    for i, (name, pwm) in enumerate(zip(ORDER, pwm_values[:6])):
        color = (0, 255, 100) if name != "Wrist" else (0, 200, 255)
        cv2.putText(frame, f"{name:<8}: PWM = {pwm:>4}", (panel_x + 5, panel_y + line_h * (i + 2)), font, 0.58, color, 1)

    # Status diody na panelu (z 7 wartości tablicy)
    led_status = pwm_values[6] if len(pwm_values) > 6 else 0
    led_color = (0, 255, 0) if led_status == 1 else (0, 0, 255)
    cv2.putText(frame, f"LED Status: {'ON (1)' if led_status == 1 else 'OFF (0)'}",
                (panel_x + 5, panel_y + line_h * (len(ORDER) + 2)), font, 0.58, led_color, 1)

    cv2.putText(frame, f"Wrist pron. : {wrist_angle:>6.3f} (-1 to 1)",
                (panel_x + 5, panel_y + line_h * (len(ORDER) + 3)), font, 0.58, (0, 180, 255), 1)


def draw_status(frame, mode, hand_detected, is_right, time_passed, h):
    font = cv2.FONT_HERSHEY_SIMPLEX
    if mode == "IDLE":
        if is_right:
            time_left = max(0.0, 2.0 - time_passed)
            cv2.putText(frame, f"  RIGHT HAND - Activating in {time_left:.1f}s  ", (10, h - 15), font, 0.65,
                        (0, 255, 255), 2)
        else:
            cv2.putText(frame, "  IDLE (FIST) - Show RIGHT hand  ", (10, h - 15), font, 0.65, (0, 60, 220), 2)
    elif mode == "TRACKING":
        if hand_detected and is_right:
            cv2.putText(frame, "  TRACKING RIGHT HAND  ", (10, h - 15), font, 0.65, (0, 200, 50), 2)
        elif hand_detected and not is_right:
            cv2.putText(frame, "  WRONG HAND (LEFT) - Ignoring  ", (10, h - 15), font, 0.65, (0, 100, 255), 2)
        else:
            time_left = max(0.0, TIMEOUT_SECONDS - time_passed)
            cv2.putText(frame, f"  NO HAND - Returning to FIST in {time_left:.1f}s  ", (10, h - 15), font, 0.65,
                        (0, 160, 255), 2)


# =============================================================
#  GŁÓWNA PĘTLA
# =============================================================
if __name__ == "__main__":
    if DEBUG: print("--- START SYSTEMU ---")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        if DEBUG: print("[BŁĄD] Nie mogę otworzyć kamery!")
        sys.exit(1)
    else:
        if DEBUG: print("[OK] Kamera otwarta.")

    hands = mp_hands.Hands(
        static_image_mode=False, max_num_hands=1,
        model_complexity=0, min_detection_confidence=0.7, min_tracking_confidence=0.6
    )

    smoothers = {name: AngleSmoother(alpha=FINGER_ALPHA, deadband=FINGER_DEADBAND) for name in SERVO_MAP.keys()}
    smoothers["Wrist"] = AngleSmoother(alpha=WRIST_ALPHA, deadband=WRIST_DEADBAND)

    returner = ReturnSmoother(alpha=RETURN_ALPHA)

    for name, val in zip(ORDER, get_fist_pwm()):
        returner.init_value(name, val)

    wrist_angle_display = 0.0
    last_hand_time = time.time()
    activation_start_time = 0.0
    frame_count = 0
    MODE = "IDLE"

    failed_frames_count = 0

    try:
        while True:
            ok, frame = cap.read()

            if not ok:
                failed_frames_count += 1
                if DEBUG: print(f"[WARN] Brak klatki z kamery! (Błąd {failed_frames_count}/30)")
                time.sleep(0.1)

                if failed_frames_count > 30:
                    if DEBUG: print("[SYSTEM] Próbuję zrestartować połączenie z kamerą...")
                    cap.release()
                    time.sleep(1.0)
                    cap = cv2.VideoCapture(0)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    failed_frames_count = 0
                continue

            failed_frames_count = 0

            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(rgb)

            output_values = []
            hand_detected = False
            is_right = False
            lm = None

            if res.multi_hand_landmarks:
                handedness_label = res.multi_handedness[0].classification[0].label
                is_right = (handedness_label == "Right")
                lm = res.multi_hand_landmarks[0].landmark
                if is_right:
                    last_hand_time = time.time()

            if MODE == "IDLE":
                if is_right:
                    hand_detected = True
                    if activation_start_time == 0.0:
                        activation_start_time = time.time()
                    else:
                        time_passed = time.time() - activation_start_time
                        if time_passed >= 2.0:
                            run_calibration_wave(sock)

                            for _ in range(15):
                                cap.grab()

                            MODE = "TRACKING"
                            last_hand_time = time.time()
                            activation_start_time = 0.0

                            for name, val in zip(ORDER, get_open_pwm()):
                                returner.set_value(name, val)
                            continue
                else:
                    if activation_start_time > 0 and (time.time() - last_hand_time) > 0.5:
                        activation_start_time = 0.0

                for name, target_val in zip(ORDER, get_fist_pwm()):
                    output_values.append(int(returner.update_toward_default(name, target_val)))

            elif MODE == "TRACKING":
                if is_right:
                    hand_detected = True
                    last_hand_time = time.time()

                    wrist_pronation = get_wrist_pronation_3d(lm, is_right_hand=True)
                    wrist_smoothed = smoothers["Wrist"].update(wrist_pronation)
                    wrist_angle_display = wrist_smoothed
                    wrist_val = map_smart(wrist_smoothed, SERVO_MAP["Wrist"])

                    rotation_angle = get_hand_rotation_angle(lm)
                    norm_lm = normalize_landmarks(lm, rotation_angle)

                    for name in ["Thumb", "Index", "Middle", "Ring", "Pinky"]:
                        p1, p2, p3 = FINGERS_INDEXES[name]
                        angle = calculate_finger_angle_from_normalized(norm_lm, p1, p2, p3)
                        smooth = smoothers[name].update(angle)
                        val = map_smart(smooth, SERVO_MAP[name])
                        output_values.append(val)
                        returner.set_value(name, val)

                    output_values.append(wrist_val)
                    returner.set_value("Wrist", wrist_val)

                else:
                    time_since_last_hand = time.time() - last_hand_time
                    if time_since_last_hand > TIMEOUT_SECONDS:
                        MODE = "IDLE"
                        output_values = [int(returner.current[n]) for n in ORDER]
                    else:
                        output_values = [int(returner.current[n]) for n in ORDER]

            # --- DODANIE STANU DIODY DO PACZKI UDP ---
            # Świeci (1) tylko wtedy, gdy poprawnie śledzi prawą dłoń
            dioda_stan = 1 if is_right else 0
            output_values.append(dioda_stan)

            # --- KOMUNIKACJA UDP (ZABEZPIECZONA) ---
            msg_str = ",".join(map(str, output_values))
            try:
                sock.sendto(msg_str.encode(), (UDP_IP, UDP_PORT))
            except Exception:
                pass

            # --- WIDOCZNOŚĆ DANYCH W KONSOLI ---
            frame_count += 1
            if DEBUG and frame_count % 15 == 0:
                print(f"[{MODE}] UDP: {msg_str}")

            # --- OBSŁUGA GUI / DEBUG ---
            if DEBUG:
                if lm is not None:
                    mp_drawing.draw_landmarks(
                        frame, res.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style()
                    )
                    draw_hand_axis(frame, lm, w, h)
                    draw_wrist_outline(frame, lm, w, h)

                draw_pwm_panel(frame, output_values, wrist_angle_display)

                if MODE == "IDLE":
                    t_pass = (time.time() - activation_start_time) if activation_start_time > 0 else 0
                    draw_status(frame, MODE, hand_detected, is_right, t_pass, h)
                else:
                    t_pass = time.time() - last_hand_time if not hand_detected else 0.0
                    draw_status(frame, MODE, hand_detected, is_right, t_pass, h)

                cv2.imshow("Hand Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        if DEBUG: print("\nOtrzymano sygnał zamknięcia. Bezpieczne zamykanie...")
    finally:
        cap.release()
        if DEBUG: cv2.destroyAllWindows()

        try:
            # --- DODANE: Zgaszenie diody (,0 na końcu) przy zamykaniu programu ---
            fist = ",".join(map(str, get_fist_pwm())) + ",0"
            fist_bytes = fist.encode()
            if DEBUG: print("[System] Zamykam rękę - spamuję LabVIEW pozycją pięści oraz gaszę diodę...")

            t_end = time.time() + 1.5
            while time.time() < t_end:
                try:
                    sock.sendto(fist_bytes, (UDP_IP, UDP_PORT))
                except Exception:
                    pass
                time.sleep(0.05)

            for _ in range(3):
                try:
                    sock.sendto(b"STOP", (UDP_IP, UDP_PORT))
                except Exception:
                    pass
                time.sleep(0.1)

            if DEBUG: print("[System] Wysłano STOP do LabVIEW.")
        except Exception:
            pass

        sock.close()