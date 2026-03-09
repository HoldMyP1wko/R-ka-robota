import os
import cv2
import mediapipe as mp
import numpy as np
import time

import board
import busio
from adafruit_pca9685 import PCA9685

# =============================================================
# WYŁĄCZENIE GUI (dla Raspberry Pi Lite)
# =============================================================
os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["GLOG_minloglevel"] = "2"

# =============================================================
# FILTRY
# =============================================================
FINGER_ALPHA = 0.15
FINGER_DEADBAND = 1.5
WRIST_ALPHA = 0.30
WRIST_DEADBAND = 0.05
RETURN_ALPHA = 0.05
TIMEOUT_SECONDS = 3.0

mp_hands = mp.solutions.hands

# =============================================================
# PCA9685 SERVO DRIVER
# =============================================================
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

SERVO_CHANNELS = {
    "Thumb": 1, #1
    "Index": 4, #4
    "Middle": 3, #3
    "Ring": 2, #2
    "Pinky": 0, #0
    "Wrist": 5
}

# =============================================================
# MAPOWANIE SERW
# =============================================================
SERVO_MAP = {
    "Thumb": {"open_pwm": 100, "close_pwm": 500, "input_min": 110, "input_max": 177},
    "Index": {"open_pwm": 100, "close_pwm": 440, "input_min": 15, "input_max": 177},
    "Middle": {"open_pwm": 525, "close_pwm": 100, "input_min": 16, "input_max": 179},
    "Ring": {"open_pwm": 100, "close_pwm": 430, "input_min": 18, "input_max": 176},
    "Pinky": {"open_pwm": 100, "close_pwm": 390, "input_min": 33, "input_max": 171},
    "Wrist": {"open_pwm": 110, "close_pwm": 600, "input_min": 0.10, "input_max": 0.85}
}

ORDER = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Wrist"]

# =============================================================
# FUNKCJE SERW
# =============================================================
def set_servo_pwm(name, pwm):
    channel = SERVO_CHANNELS[name]
    duty = int(np.interp(pwm, [0, 600], [1500, 8000]))
    pca.channels[channel].duty_cycle = duty

def apply_servo_positions(values):
    for name, val in zip(ORDER, values):
        set_servo_pwm(name, val)

def get_fist_pwm():
    return [SERVO_MAP[name]["open_pwm"] if name == "Wrist" else SERVO_MAP[name]["close_pwm"] for name in ORDER]

def get_open_pwm():
    return [SERVO_MAP[name]["open_pwm"] for name in ORDER]

# =============================================================
# FILTRY
# =============================================================
class AngleSmoother:
    def __init__(self, alpha=0.4, deadband=2.0):
        self.alpha = alpha
        self.dead = deadband
        self.ema = None
        self.last_target = None

    def update(self, new_val):
        if self.ema is None:
            self.ema = new_val
            self.last_target = new_val
            return new_val

        if abs(new_val - self.last_target) > self.dead:
            self.last_target = new_val

        self.ema = (1 - self.alpha) * self.ema + self.alpha * self.last_target
        return self.ema

class ReturnSmoother:
    def __init__(self, alpha=0.05):
        self.alpha = alpha
        self.current = {}

    def init_value(self, name, default_val):
        self.current[name] = default_val

    def update_toward_default(self, name, default_val):
        self.current[name] = (1-self.alpha)*self.current[name] + self.alpha*default_val
        return self.current[name]

    def set_value(self, name, val):
        self.current[name] = val

# =============================================================
# GEOMETRIA
# =============================================================
FINGERS_INDEXES = {
    "Thumb": (1,2,4),
    "Index": (5,6,8),
    "Middle": (9,10,12),
    "Ring": (13,14,16),
    "Pinky": (17,18,20),
}

def get_hand_rotation_angle(lm):
    wrist = np.array([lm[0].x,lm[0].y])
    middle = np.array([lm[9].x,lm[9].y])
    axis = middle - wrist
    return np.degrees(np.arctan2(axis[0],-axis[1]))

def normalize_landmarks(lm,angle):
    rad=np.radians(angle)
    cos=np.cos(-rad)
    sin=np.sin(-rad)
    rot=np.array([[cos,-sin],[sin,cos]])
    origin=np.array([lm[0].x,lm[0].y])
    return [rot@(np.array([p.x,p.y])-origin) for p in lm]

def calc_angle(norm,p1,p2,p3):
    a,b,c=norm[p1],norm[p2],norm[p3]
    ba=a-b
    bc=c-b
    cos=np.dot(ba,bc)/(np.linalg.norm(ba)*np.linalg.norm(bc))
    cos=np.clip(cos,-1,1)
    return np.degrees(np.arccos(cos))

def get_wrist_pronation(lm):
    ux,uy,uz=lm[5].x-lm[0].x,lm[5].y-lm[0].y,lm[5].z-lm[0].z
    vx,vy,vz=lm[17].x-lm[0].x,lm[17].y-lm[0].y,lm[17].z-lm[0].z
    nx=uy*vz-uz*vy
    nz=uz*vy-uy*vx
    return np.clip(np.arctan2(nx,-nz)/np.pi,-1,1)

def map_smart(value,cfg):
    norm=(np.clip(value,cfg["input_min"],cfg["input_max"])-cfg["input_min"]) /(cfg["input_max"]-cfg["input_min"])
    return int(cfg["close_pwm"]+norm*(cfg["open_pwm"]-cfg["close_pwm"]))

# =============================================================
# START
# =============================================================
print("START SYSTEMU")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Ustawienie max_num_hands=1 dla wydajności
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=0
)

smoothers = {name: AngleSmoother(FINGER_ALPHA, FINGER_DEADBAND) for name in SERVO_MAP}
returner = ReturnSmoother(RETURN_ALPHA)

for n, v in zip(ORDER, get_fist_pwm()):
    returner.init_value(n, v)

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)  # Poczekaj na kamerę
            continue

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)

        output = []

        if res.multi_hand_landmarks:
            lm = res.multi_hand_landmarks[0].landmark

            wrist_val = map_smart(
                smoothers["Wrist"].update(get_wrist_pronation(lm)),
                SERVO_MAP["Wrist"]
            )

            rot = get_hand_rotation_angle(lm)
            norm = normalize_landmarks(lm, rot)

            for name in ["Thumb", "Index", "Middle", "Ring", "Pinky"]:
                p1, p2, p3 = FINGERS_INDEXES[name]
                angle = calc_angle(norm, p1, p2, p3)
                val = map_smart(smoothers[name].update(angle), SERVO_MAP[name])
                output.append(val)
                returner.set_value(name, val)

            output.append(wrist_val)
            apply_servo_positions(output)

        else:
            # Płynny powrót do pięści (lub pozycji domyślnej) gdy zgubi dłoń
            for name, val in zip(ORDER, get_fist_pwm()):
                val = int(returner.update_toward_default(name, val))
                output.append(val)

            apply_servo_positions(output)

        # Małe opóźnienie żeby procesor nie ugotował się na 100% użycia
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Zatrzymywanie programu...")
finally:
    # Uwolnij kamerę przy zamknięciu programu
    cap.release()
    print("Kamera zwolniona, dobranoc!")