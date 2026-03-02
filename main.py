import os
import cv2
import mediapipe as mp
import numpy as np
import socket
import time
import math

# =============================================================
#  FLAGA DEBUGOWANIA
#  Zmień na False, gdy wgrywasz na Raspberry Pi na wystawkę!
# =============================================================
DEBUG = True
# =============================================================

# =============================================================
#  CZUŁOŚĆ I FILTRY
# =============================================================
FINGER_ALPHA = 0.15
FINGER_DEADBAND = 1.5

WRIST_ALPHA = 0.30
WRIST_DEADBAND = 0.05
# =============================================================

# =============================================================
#  POWRÓT DO POZYCJI DOMYŚLNEJ I BEZPIECZEŃSTWO
# =============================================================
RETURN_ALPHA = 0.05
TIMEOUT_SECONDS = 3.0  # Czas po jakim dłoń wraca do pozycji otwartej (sekundy)
# =============================================================

# --- KONFIGURACJA UDP ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

os.environ["GLOG_minloglevel"] = "2"

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# --- DEFINICJA PALCÓW ---
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
        "open_pwm": 100, "close_pwm": 320,
        "input_min": 110, "input_max": 177
    },
    "Index": {
        "open_pwm": 120, "close_pwm": 380,
        "input_min": 15, "input_max": 177
    },
    "Middle": {
        "open_pwm": 100, "close_pwm": 480,
        "input_min": 16, "input_max": 179
    },
    "Ring": {
        "open_pwm": 630, "close_pwm": 340,
        "input_min": 18, "input_max": 176
    },
    "Pinky": {
        "open_pwm": 240, "close_pwm": 510,
        "input_min": 33, "input_max": 171
    },
    "Wrist": {
        "open_pwm": 110, "close_pwm": 330,
        "input_min": -1.0, "input_max": 1.0
    }
}

ORDER = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Wrist"]


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
    rot = np.array([[cos_a, -sin_a],
                    [sin_a, cos_a]])
    origin = np.array([lm[0].x, lm[0].y])
    normalized = []
    for point in lm:
        p = np.array([point.x, point.y]) - origin
        p_rot = rot @ p
        normalized.append(p_rot)
    return normalized


def calculate_finger_angle_from_normalized(norm_lm, p1, p2, p3):
    a = norm_lm[p1]
    b = norm_lm[p2]
    c = norm_lm[p3]
    ba = a - b
    bc = c - b
    norm_ba = np.linalg.norm(ba)
    norm_bc = np.linalg.norm(bc)
    if norm_ba == 0 or norm_bc == 0:
        return 0.0
    cosine = np.dot(ba, bc) / (norm_ba * norm_bc)
    angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
    return angle


def get_wrist_pronation_3d(lm, is_right_hand=True):
    ux = lm[5].x - lm[0].x
    uy = lm[5].y - lm[0].y
    uz = lm[5].z - lm[0].z

    vx = lm[17].x - lm[0].x
    vy = lm[17].y - lm[0].y
    vz = lm[17].z - lm[0].z

    nx = uy * vz - uz * vy
    nz = ux * vy - uy * vx

    if not is_right_hand:
        nx = -nx
        nz = -nz

    angle_rad = np.arctan2(nx, -nz)
    pronation = angle_rad / np.pi
    return float(np.clip(pronation, -1.0, 1.0))


def map_smart(value, cfg):
    in_min = cfg["input_min"]
    in_max = cfg["input_max"]
    pwm_open = cfg["open_pwm"]
    pwm_close = cfg["close_pwm"]
    val = max(in_min, min(value, in_max))
    norm = (val - in_min) / (in_max - in_min)
    out = pwm_close + norm * (pwm_open - pwm_close)
    return int(out)


def get_defaults():
    return [SERVO_MAP[name]["open_pwm"] for name in ORDER]


# =============================================================
#  FUNKCJE DEBUGOWANIA
# =============================================================

def draw_hand_axis(frame, lm, w, h):
    x0 = int(lm[0].x * w)
    y0 = int(lm[0].y * h)
    x9 = int(lm[9].x * w)
    y9 = int(lm[9].y * h)
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


def draw_pwm_panel(frame, pwm_values, wrist_angle, hand_detected):
    panel_x = 10
    panel_y = 10
    line_h = 26
    font = cv2.FONT_HERSHEY_SIMPLEX

    panel_w = 270
    panel_h = line_h * (len(ORDER) + 3) + 10

    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    cv2.putText(frame, "=== UDP OUTPUT ===", (panel_x + 5, panel_y + line_h), font, 0.5, (0, 255, 255), 1)

    for i, (name, pwm) in enumerate(zip(ORDER, pwm_values)):
        y_pos = panel_y + line_h * (i + 2)
        color = (0, 255, 100) if name != "Wrist" else (0, 200, 255)
        if not hand_detected: color = tuple(int(c * 0.5) for c in color)
        cv2.putText(frame, f"{name:<8}: PWM = {pwm:>4}", (panel_x + 5, y_pos), font, 0.58, color, 1)

    y_angle = panel_y + line_h * (len(ORDER) + 2)
    cv2.putText(frame, f"Wrist pron. : {wrist_angle:>6.3f} (-1 to 1)", (panel_x + 5, y_angle), font, 0.58,
                (0, 180, 255), 1)


def draw_status(frame, hand_detected, time_since_last_hand, h):
    font = cv2.FONT_HERSHEY_SIMPLEX
    if hand_detected:
        cv2.putText(frame, "  HAND DETECTED  ", (10, h - 15), font, 0.65, (0, 200, 50), 2)
    else:
        if time_since_last_hand > TIMEOUT_SECONDS:
            cv2.putText(frame, "  NO HAND - returning to default  ", (10, h - 15), font, 0.65, (0, 60, 220), 2)
        else:
            time_left = TIMEOUT_SECONDS - time_since_last_hand
            cv2.putText(frame, f"  NO HAND - waiting... {time_left:.1f}s  ", (10, h - 15), font, 0.65, (0, 160, 255), 2)


# =============================================================
#  GŁÓWNA PĘTLA
# =============================================================

if __name__ == "__main__":
    if DEBUG:
        print(f"--- START SYSTEMU W TRYBIE DEBUG ---")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        model_complexity=0,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.6
    )

    smoothers = {name: AngleSmoother(alpha=FINGER_ALPHA, deadband=FINGER_DEADBAND) for name in SERVO_MAP.keys()}
    smoothers["Wrist"] = AngleSmoother(alpha=WRIST_ALPHA, deadband=WRIST_DEADBAND)

    returner = ReturnSmoother(alpha=RETURN_ALPHA)
    defaults = get_defaults()
    for name, val in zip(ORDER, defaults):
        returner.init_value(name, val)

    wrist_angle_display = 0.0
    last_hand_time = time.time()
    frame_count = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok: continue

            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(rgb)

            output_values = []
            hand_detected = False

            if res.multi_hand_landmarks:
                hand_detected = True
                last_hand_time = time.time()

                lm = res.multi_hand_landmarks[0].landmark

                handedness_label = res.multi_handedness[0].classification[0].label
                is_right = (handedness_label == "Right")

                # --- NADGARSTEK ---
                wrist_pronation = get_wrist_pronation_3d(lm, is_right_hand=is_right)
                wrist_smoothed = smoothers["Wrist"].update(wrist_pronation)
                wrist_angle_display = wrist_smoothed
                wrist_val = map_smart(wrist_smoothed, SERVO_MAP["Wrist"])

                # --- PALCE ---
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

                # Rysowanie na ekranie TYLKO jak DEBUG jest True
                if DEBUG:
                    mp_drawing.draw_landmarks(
                        frame, res.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style()
                    )
                    draw_hand_axis(frame, lm, w, h)
                    draw_wrist_outline(frame, lm, w, h)

            else:
                time_since_last_hand = time.time() - last_hand_time

                if time_since_last_hand > TIMEOUT_SECONDS:
                    for name, default_val in zip(ORDER, get_defaults()):
                        output_values.append(int(returner.update_toward_default(name, default_val)))
                    if hasattr(draw_wrist_outline, "prev_p1"):
                        draw_wrist_outline.prev_p1 = None
                else:
                    for name in ORDER:
                        output_values.append(int(returner.current[name]))

            # Obsługa GUI, klawiszy i LOGÓW TYLKO w trybie DEBUG
            if DEBUG:
                time_passed = time.time() - last_hand_time if not hand_detected else 0.0
                draw_pwm_panel(frame, output_values, wrist_angle_display if hand_detected else 0.0, hand_detected)
                draw_status(frame, hand_detected, time_passed, h)
                cv2.imshow("Hand Tracking", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Co ok. sekundę rzucamy loga, żeby widzieć że działa
                frame_count += 1
                if frame_count % 30 == 0:
                    msg_str_debug = ",".join(map(str, output_values))
                    print(f"UDP SEND: {msg_str_debug}")

            # --- KOMUNIKACJA UDP ---
            msg_str = ",".join(map(str, output_values))
            sock.sendto(msg_str.encode(), (UDP_IP, UDP_PORT))

    except KeyboardInterrupt:
        if DEBUG:
            print("\nOtrzymano sygnał zamknięcia. Bezpieczne zamykanie...")
    finally:
        cap.release()
        if DEBUG:
            cv2.destroyAllWindows()
            print("System wyłączony.")
        sock.close()