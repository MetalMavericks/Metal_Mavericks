import time
from enum import Enum, auto
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# --- Hardware Pin Assignments ---
PIN_DIR_MOTOR = 17
PIN_PWM_MOTOR = 27
PIN_PWM_SERVO = 25

# --- Frame & Resolution Dimensions ---
FRAME_WIDTH = 1920
FRAME_HEIGHT = 880
HALF_WIDTH = FRAME_WIDTH // 2

# --- Steering Limits & Proportional Control ---
STEER_CENTER = 95
STEER_LEFT_LIMIT = 70
STEER_RIGHT_LIMIT = 125
KP_GAIN = 0.02
DEFAULT_THROTTLE = 40

# --- Navigation Target Rules ---
GATE_COOLDOWN_SEC = 1.2
TARGET_GATE_COUNT = 12


class SteeringDirection(Enum):
    UNSET = auto()
    CLOCKWISE = auto()
    COUNTER_CLOCKWISE = auto()


class VehicleDriver:
    """Manages low-level pin configurations for driving and steering actuators."""

    def __init__(self, dir_pin, motor_pwm_pin, servo_pwm_pin):
        self.dir_pin = dir_pin
        self.motor_pwm_pin = motor_pwm_pin
        self.servo_pwm_pin = servo_pwm_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in [self.dir_pin, self.motor_pwm_pin, self.servo_pwm_pin]:
            GPIO.setup(pin, GPIO.OUT)

        self.drive_pwm = GPIO.PWM(self.motor_pwm_pin, 1000)
        self.steer_pwm = GPIO.PWM(self.servo_pwm_pin, 50)

        self.drive_pwm.start(0)
        self.steer_pwm.start(0)

    def set_steering_angle(self, target_angle):
        clamped_angle = max(STEER_LEFT_LIMIT, min(STEER_RIGHT_LIMIT, target_angle))
        duty_cycle = 2.5 + (clamped_angle / 180.0) * 10.0

        self.steer_pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.05)
        self.steer_pwm.ChangeDutyCycle(0)

    def set_motor_speed(self, speed_percent):
        GPIO.output(self.dir_pin, GPIO.HIGH)
        self.drive_pwm.ChangeDutyCycle(speed_percent)

    def stop_motion(self):
        self.drive_pwm.ChangeDutyCycle(0)

    def release_hardware(self):
        self.stop_motion()
        self.drive_pwm.stop()
        self.steer_pwm.stop()
        GPIO.cleanup()


class VisionPipeline:
    """Handles HSV color segmentation, morphology, and landmark extraction."""

    # Color Thresholds in Standard HSV Space (Hue: 0-180, Sat: 0-255, Val: 0-255)
    HSV_BLACK = (np.array([0, 0, 0]), np.array([180, 255, 60]))
    HSV_BLUE = (np.array([100, 150, 50]), np.array([140, 255, 255]))
    HSV_ORANGE = (np.array([5, 150, 150]), np.array([25, 255, 255]))
    HSV_GREEN = (np.array([40, 70, 70]), np.array([85, 255, 255]))
    HSV_RED = (np.array([0, 120, 120]), np.array([10, 255, 255]))

    def __init__(self):
        self.clahe_filter = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        self.morph_kernel = np.ones((5, 5), np.uint8)
        self.large_kernel = np.ones((15, 15), np.uint8)

    def process_frame(self, raw_frame):
        blurred = cv2.GaussianBlur(raw_frame, (5, 5), 0)
        hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Equalize Value Channel for constant exposure tracking
        h, s, v = cv2.split(hsv_image)
        v = self.clahe_filter.apply(v)
        hsv_image = cv2.merge((h, s, v))

        annotated_frame = raw_frame.copy()

        # Extract Targets
        left_line, right_line = self._find_black_borders(hsv_image, annotated_frame)
        green_point = self._find_pillar_target(hsv_image, self.HSV_GREEN, (30000, 150000), (0, 255, 0), annotated_frame, False)
        red_point = self._find_pillar_target(hsv_image, self.HSV_RED, (800, float('inf')), (0, 0, 255), annotated_frame, True)

        sees_blue = self._detect_marker_presence(hsv_image, self.HSV_BLUE, min_area=800, check_x_threshold=300)
        sees_orange = self._detect_orange_cluster(hsv_image, min_area=500)

        return (left_line, right_line, green_point, red_point, sees_blue, sees_orange), annotated_frame

    def _find_black_borders(self, hsv_img, canvas):
        mask = cv2.inRange(hsv_img, self.HSV_BLACK[0], self.HSV_BLACK[1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        left_target, right_target = None, None
        max_left_y, max_right_y = -1, -1

        for c in contours:
            if cv2.contourArea(c) < 3000:
                continue

            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (255, 255, 0), 2)

            center_x = x + (w // 2)
            bottom_y = y + h

            if center_x < HALF_WIDTH:
                if bottom_y > max_left_y:
                    max_left_y = bottom_y
                    left_target = (x + w, bottom_y)
                    cv2.circle(canvas, left_target, 8, (255, 0, 0), -1)
            else:
                if bottom_y > max_right_y:
                    max_right_y = bottom_y
                    right_target = (x, bottom_y)
                    cv2.circle(canvas, right_target, 8, (0, 0, 255), -1)

        return left_target, right_target

    def _find_pillar_target(self, hsv_img, bounds, area_range, color_bgr, canvas, is_red):
        mask = cv2.inRange(hsv_img, bounds[0], bounds[1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area_range[0] < area < area_range[1]:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), color_bgr, 2)

            target_point = (x + w, y + h) if is_red else (x, y + h)
            cv2.circle(canvas, target_point, 8, (0, 255, 255), -1)
            return target_point

        return None

    def _detect_marker_presence(self, hsv_img, bounds, min_area, check_x_threshold):
        mask = cv2.inRange(hsv_img, bounds[0], bounds[1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > min_area:
            x, y, w, h = cv2.boundingRect(largest)
            return (x + w) > check_x_threshold

        return False

    def _detect_orange_cluster(self, hsv_img, min_area):
        mask = cv2.inRange(hsv_img, self.HSV_ORANGE[0], self.HSV_ORANGE[1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.large_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > 50]

        if valid_contours:
            total_area = sum(cv2.contourArea(c) for c in valid_contours)
            return total_area > min_area

        return False


class CourseNavigator:
    """Controls direction orientation state machine and obstacle arbitration."""

    def __init__(self, target_gates):
        self.direction = SteeringDirection.UNSET
        self.gate_counter = 0
        self.target_gates = target_gates
        self.last_blue_ts = 0.0
        self.last_orange_ts = 0.0

    def update_gate_tracking(self, sees_blue, sees_orange):
        now = time.time()

        if self.direction == SteeringDirection.UNSET:
            if sees_blue:
                self.direction = SteeringDirection.COUNTER_CLOCKWISE
                print("[SYSTEM]: Direction set -> COUNTER-CLOCKWISE")
            elif sees_orange:
                self.direction = SteeringDirection.CLOCKWISE
                print("[SYSTEM]: Direction set -> CLOCKWISE")

        elif self.direction == SteeringDirection.CLOCKWISE:
            if sees_blue and (now - self.last_blue_ts > GATE_COOLDOWN_SEC):
                self.gate_counter += 1
                self.last_blue_ts = now
                print(f"[NAV]: Passed Gate {self.gate_counter}/{self.target_gates}")

        elif self.direction == SteeringDirection.COUNTER_CLOCKWISE:
            if sees_orange and (now - self.last_orange_ts > GATE_COOLDOWN_SEC):
                self.gate_counter += 1
                self.last_orange_ts = now
                print(f"[NAV]: Passed Gate {self.gate_counter}/{self.target_gates}")

    def calculate_steering_angle(self, detections):
        left_line, right_line, green_point, red_point, _, _ = detections
        is_clockwise = (self.direction == SteeringDirection.CLOCKWISE)

        # Priority 1: Green Pillar Avoidance
        if green_point:
            gx, gy = green_point
            if is_clockwise or gy > 500:
                offset = gx - (FRAME_WIDTH - 100)
            else:
                offset = gx - (FRAME_WIDTH - 560)
            return STEER_CENTER + (offset * KP_GAIN)

        # Priority 2: Red Pillar Avoidance
        if red_point:
            rx, ry = red_point
            if not is_clockwise or ry > 500:
                offset = rx - 100
            else:
                offset = rx - 560
            return STEER_CENTER + (offset * KP_GAIN)

        # Priority 3: Dual Boundary Tracking
        if left_line and right_line:
            left_dist = left_line[0]
            right_dist = FRAME_WIDTH - right_line[0]
            return STEER_CENTER + (left_dist - right_dist) * KP_GAIN

        # Priority 4: Single Boundary Fallback
        if left_line:
            only_x = left_line[0]
            offset = only_x if is_clockwise else (only_x - 200)
            return STEER_CENTER + (offset * KP_GAIN)

        if right_line:
            only_x = right_line[0]
            offset = (only_x - (FRAME_WIDTH - 200)) if is_clockwise else (only_x - FRAME_WIDTH)
            return STEER_CENTER + (offset * KP_GAIN)

        # Priority 5: Open Loop Recovery Angle
        return STEER_CENTER + (15 if is_clockwise else -15)


def initialize_camera():
    cam = Picamera2()
    config = cam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    cam.configure(config)
    cam.start()

    cam.set_controls({"AeEnable": True, "AwbEnable": True})
    time.sleep(2.0)

    meta = cam.capture_metadata()
    cam.set_controls({
        "AeEnable": False,
        "AwbEnable": False,
        "ExposureTime": meta["ExposureTime"],
        "AnalogueGain": meta["AnalogueGain"]
    })
    return cam


def main():
    driver = VehicleDriver(PIN_DIR_MOTOR, PIN_PWM_MOTOR, PIN_PWM_SERVO)
    vision = VisionPipeline()
    navigator = CourseNavigator(TARGET_GATE_COUNT)

    cam = initialize_camera()

    try:
        driver.set_steering_angle(STEER_CENTER)
        time.sleep(1.0)
        driver.set_motor_speed(DEFAULT_THROTTLE)

        print("[SYSTEM]: Main loop active.")

        while True:
            frame = cam.capture_array()
            detections, debug_view = vision.process_frame(frame)

            navigator.update_gate_tracking(detections[4], detections[5])

            if navigator.gate_counter >= TARGET_GATE_COUNT:
                print(f"[SYSTEM]: Target gates reached ({TARGET_GATE_COUNT}). Terminating.")
                driver.set_steering_angle(STEER_CENTER)
                driver.stop_motion()
                break

            angle = navigator.calculate_steering_angle(detections)
            driver.set_steering_angle(angle)

            cv2.imshow("Navigation Diagnostics", debug_view)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[SYSTEM]: User interrupted execution.")
                driver.set_steering_angle(STEER_CENTER)
                driver.stop_motion()
                break

    finally:
        driver.release_hardware()
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
