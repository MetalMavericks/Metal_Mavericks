import time
from enum import Enum, auto
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# --- Hardware Configuration Pinout ---
PIN_MOTOR_IN1 = 21
PIN_MOTOR_IN2 = 16
PIN_PWM_MOTOR = 12
PIN_PWM_SERVO = 13

# --- Frame & ROI Parameters ---
FRAME_WIDTH = 1080
FRAME_HEIGHT = 680
MID_X = FRAME_WIDTH // 2

CROP_TOP_ROW = 120
EDGE_MARGIN = int(FRAME_WIDTH * 0.10)

# --- Navigation & Control Config ---
TARGET_AREA_MIN = 600
PROPORTIONAL_GAIN = 0.04
DRIVE_SPEED = 50

ANGLE_CENTER = 95
ANGLE_MIN_RIGHT = 30
ANGLE_MAX_LEFT = 150

# --- Lap Tracker Config ---
TARGET_LAPS = 3
CROSSINGS_PER_LAP = 4
TOTAL_REQUIRED_CROSSINGS = TARGET_LAPS * CROSSINGS_PER_LAP
GATE_COOLDOWN_SEC = 2.1


class TrackDirection(Enum):
    UNKNOWN = auto()
    CLOCKWISE = auto()
    COUNTER_CLOCKWISE = auto()


class MotorController:
    """Handles low-level GPIO pin driving for steering servo and main traction motor."""
    
    def __init__(self, pin_in1, pin_in2, pin_motor_pwm, pin_servo_pwm):
        self.pin_in1 = pin_in1
        self.pin_in2 = pin_in2
        self.pin_motor_pwm = pin_motor_pwm
        self.pin_servo_pwm = pin_servo_pwm

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        for p in [self.pin_in1, self.pin_in2, self.pin_motor_pwm, self.pin_servo_pwm]:
            GPIO.setup(p, GPIO.OUT)

        self.m_pwm = GPIO.PWM(self.pin_motor_pwm, 1000)
        self.s_pwm = GPIO.PWM(self.pin_servo_pwm, 50)
        
        self.m_pwm.start(0)
        self.s_pwm.start(0)

    def set_steering(self, target_angle):
        clamped_angle = max(ANGLE_MIN_RIGHT, min(ANGLE_MAX_LEFT, target_angle))
        duty_cycle = 2.5 + (clamped_angle / 180.0) * 10.0
        
        self.s_pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.01)
        self.s_pwm.ChangeDutyCycle(0)

    def drive_forward(self, throttle_percent):
        GPIO.output(self.pin_in1, GPIO.HIGH)
        GPIO.output(self.pin_in2, GPIO.LOW)
        self.m_pwm.ChangeDutyCycle(throttle_percent)

    def halt(self):
        self.m_pwm.ChangeDutyCycle(0)
        GPIO.output(self.pin_in1, GPIO.LOW)
        GPIO.output(self.pin_in2, GPIO.LOW)

    def cleanup(self):
        self.halt()
        self.s_pwm.stop()
        self.m_pwm.stop()
        GPIO.cleanup()


class CameraDriver:
    """Manages PiCamera2 lifecycle and sensor gain configuration."""
    
    def __init__(self, w, h):
        self.width = w
        self.height = h
        self.cam = Picamera2(camera_num=1)

    def start(self):
        cfg = self.cam.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.cam.configure(cfg)
        self.cam.start()

        print("Calibrating Auto-Exposure & White Balance...")
        self.cam.set_controls({"AeEnable": True, "AwbEnable": True})
        time.sleep(2.0)

        metadata = self.cam.capture_metadata()
        self.cam.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": metadata["ExposureTime"],
            "AnalogueGain": metadata["AnalogueGain"]
        })
        print(f"Exposure Locked: {metadata['ExposureTime']} µs | Gain: {metadata['AnalogueGain']}")

    def read_frame(self):
        return self.cam.capture_array()

    def stop(self):
        self.cam.stop()


class LaneDetector:
    """Handles HSV color segmentation and steering target extraction."""
    
    # Standardized HSV Color Ranges
    HSV_BLACK = (np.array([0, 0, 0]), np.array([180, 255, 60]))
    HSV_BLUE = (np.array([100, 150, 50]), np.array([140, 255, 255]))
    HSV_ORANGE = (np.array([5, 150, 150]), np.array([25, 255, 255]))

    def __init__(self):
        self.clahe_filter = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        self.morph_kernel = np.ones((5, 5), np.uint8)

    def process_frame(self, frame_bgr):
        # Preprocessing via Blur & Adaptive Contrast Enhancement
        blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
        hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Equalize V channel for illumination resistance
        h, s, v = cv2.split(hsv_frame)
        v = self.clahe_filter.apply(v)
        hsv_frame = cv2.merge((h, s, v))

        # Detect markers to ignore within wall bounds
        orange_mask = self._get_mask(hsv_frame, self.HSV_ORANGE, apply_edge_ignore=True)
        blue_mask = self._get_mask(hsv_frame, self.HSV_BLUE, apply_edge_ignore=True)

        # Detect track line while masking out orange artifacts
        black_mask = cv2.inRange(hsv_frame, self.HSV_BLACK[0], self.HSV_BLACK[1])
        black_mask[:CROP_TOP_ROW, :] = 0
        black_mask = cv2.bitwise_and(black_mask, cv2.bitwise_not(orange_mask))
        
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, self.morph_kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, self.morph_kernel)

        left_node, right_node = self._find_lane_targets(black_mask)
        
        has_blue = self._evaluate_marker_contours(blue_mask, min_size=800)
        has_orange = self._evaluate_marker_contours(orange_mask, min_size=800)

        return left_node, right_node, has_blue, has_orange

    def _get_mask(self, hsv, hsv_bounds, apply_edge_ignore=False):
        mask = cv2.inRange(hsv, hsv_bounds[0], hsv_bounds[1])
        mask[:CROP_TOP_ROW, :] = 0
        if apply_edge_ignore:
            mask[:, :EDGE_MARGIN] = 0
            mask[:, FRAME_WIDTH - EDGE_MARGIN:] = 0
        
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)

    def _find_lane_targets(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        left_target, right_target = None, None
        max_left_y, max_right_y = -1, -1

        for c in contours:
            if cv2.contourArea(c) < TARGET_AREA_MIN:
                continue
            
            x, y, w, h = cv2.boundingRect(c)
            center_x = x + (w // 2)
            bottom_y = y + h

            if center_x < MID_X:
                if bottom_y > max_left_y:
                    max_left_y = bottom_y
                    left_target = (x + w, bottom_y)
            else:
                if bottom_y > max_right_y:
                    max_right_y = bottom_y
                    right_target = (x, bottom_y)

        return left_target, right_target

    def _evaluate_marker_contours(self, mask, min_size):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False
        largest_area = max([cv2.contourArea(c) for c in contours], default=0)
        return largest_area > min_size


class LapTracker:
    """Manages direction detection state transitions and lap/gate timing."""
    
    def __init__(self, cooldown):
        self.direction = TrackDirection.UNKNOWN
        self.gate_count = 0
        self.cooldown = cooldown
        self.last_gate_timestamp = 0.0
        self.was_marker_visible = False

    def update(self, sees_blue, sees_orange):
        now = time.time()

        # Step 1: Detect Track Orientation on Initial Passing
        if self.direction == TrackDirection.UNKNOWN:
            if sees_blue:
                self.direction = TrackDirection.COUNTER_CLOCKWISE
                self.last_gate_timestamp = now
                print("[SYSTEM]: Direction locked -> COUNTER-CLOCKWISE")
            elif sees_orange:
                self.direction = TrackDirection.CLOCKWISE
                self.last_gate_timestamp = now
                print("[SYSTEM]: Direction locked -> CLOCKWISE")
            return

        # Step 2: Track Gate Crossings using Edge Triggering
        current_marker = sees_orange if self.direction == TrackDirection.CLOCKWISE else sees_blue
        
        # Debounced rising edge trigger condition
        if current_marker and not self.was_marker_visible:
            if (now - self.last_gate_timestamp) > self.cooldown:
                self.gate_count += 1
                self.last_gate_timestamp = now
                print(f"[TRACKER]: Gate Passed! Total Crossings = {self.gate_count}/{TOTAL_REQUIRED_CROSSINGS}")

        self.was_marker_visible = current_marker


def compute_steering_angle(left_target, right_target, is_inverted=False):
    """Calculates proportional steering angles based on line error vector."""
    if left_target and right_target:
        error = left_target[0] - (FRAME_WIDTH - right_target[0])
    elif left_target:
        error = left_target[0] - 200
    elif right_target:
        error = right_target[0] - (FRAME_WIDTH - 200)
    else:
        return ANGLE_CENTER

    correction = error * PROPORTIONAL_GAIN
    return ANGLE_CENTER - correction if not is_inverted else ANGLE_CENTER + correction


def main():
    camera = CameraDriver(FRAME_WIDTH, FRAME_HEIGHT)
    actuators = MotorController(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_PWM_MOTOR, PIN_PWM_SERVO)
    vision = LaneDetector()
    tracker = LapTracker(GATE_COOLDOWN_SEC)

    try:
        camera.start()
        actuators.set_steering(ANGLE_CENTER)
        time.sleep(1.5)
        actuators.drive_forward(DRIVE_SPEED)

        print(f"[MAIN]: Vehicle Active. Targeted Target Laps: {TARGET_LAPS}")

        while True:
            frame = camera.read_frame()
            left_node, right_node, blue_flag, orange_flag = vision.process_frame(frame)
            
            # Update Direction / Laps State Machine
            tracker.update(blue_flag, orange_flag)

            # Check Termination Condition
            if tracker.gate_count >= TOTAL_REQUIRED_CROSSINGS:
                print(f"[MAIN]: Finished {TARGET_LAPS} Laps ({tracker.gate_count} gates). Halting.")
                actuators.set_steering(ANGLE_CENTER)
                actuators.drive_forward(20)
                time.sleep(0.5)
                actuators.halt()
                break

            # Steering Control Routing
            invert_steering = (tracker.direction == TrackDirection.CLOCKWISE)
            target_angle = compute_steering_angle(left_node, right_node, is_inverted=invert_steering)
            actuators.set_steering(target_angle)

            # Diagnostic Status Telemetry
            completed_laps = tracker.gate_count // CROSSINGS_PER_LAP
            print(f"[TELEMETRY]: Gate {tracker.gate_count}/{TOTAL_REQUIRED_CROSSINGS} | Lap {completed_laps}/{TARGET_LAPS}", end='\r')

            # Break Execution via Keyboard Signal
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[MAIN]: Manual Interruption Requested.")
                break

    finally:
        actuators.cleanup()
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
