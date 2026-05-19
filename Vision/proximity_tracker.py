"""
Overhead camera tracker for the maze game.

Detects a coloured arrow on the RC car roof via HSV thresholding, computes
Euclidean distance to an overseer-set destination through a four-point
homography, maps distance to haptic motor parameters, and streams compact
binary frames to a bridge ESP32 over USB serial.
"""

import argparse
import struct
import sys
import time

import cv2
import numpy as np
import serial

# ---------------------------------------------------------------------------
# Serial protocol (must match ProximityBridge_ESPNow / Pot_Controller)
# ---------------------------------------------------------------------------
FRAME_MAGIC = bytes([0xAA, 0x55])
# <  = little-endian (matches ESP32)
# I  = uint32_t seq
# B  = uint8_t  strength
# H  = uint16_t pulse_on_ms
# H  = uint16_t pulse_off_ms
# B  = uint8_t  flags
PACKET_FMT = "<IBHHB"
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # 10

FLAG_VALID = 0x01

# ---------------------------------------------------------------------------
# Maze geometry (mm)
# ---------------------------------------------------------------------------
MAZE_WIDTH_MM = 1200
MAZE_HEIGHT_MM = 1200

# ---------------------------------------------------------------------------
# Distance -> motor mapping
# ---------------------------------------------------------------------------
MAX_DISTANCE_MM = 1200.0
ARRIVAL_THRESHOLD_MM = 50.0

# ---------------------------------------------------------------------------
# Arrow detection defaults (bright saturated colour on neutral roof)
# ---------------------------------------------------------------------------
DEFAULT_HSV_LOWER = (5, 150, 100)
DEFAULT_HSV_UPPER = (25, 255, 255)
MIN_CONTOUR_AREA = 200


def xor_checksum(data: bytes) -> int:
    chk = 0
    for b in data:
        chk ^= b
    return chk


def build_frame(seq: int, strength: int, pulse_on: int,
                pulse_off: int, flags: int) -> bytes:
    payload = struct.pack(PACKET_FMT, seq, strength, pulse_on, pulse_off, flags)
    return FRAME_MAGIC + payload + bytes([xor_checksum(payload)])


def distance_to_motor_params(distance_mm: float, valid: bool):
    """Map distance (mm) to (strength, pulse_on_ms, pulse_off_ms, flags)."""
    if not valid:
        return 0, 0, 0, 0

    d = max(0.0, min(distance_mm, MAX_DISTANCE_MM))

    if d < ARRIVAL_THRESHOLD_MM:
        return 200, 500, 0, FLAG_VALID

    t = (d - ARRIVAL_THRESHOLD_MM) / (MAX_DISTANCE_MM - ARRIVAL_THRESHOLD_MM)
    t = max(0.0, min(1.0, t))

    strength = int(80 + (1.0 - t) * 120)
    hz = 1.0 + (1.0 - t) * 7.0
    pulse_on = 60
    pulse_off = max(0, int(1000.0 / hz - pulse_on))

    return strength, pulse_on, pulse_off, FLAG_VALID


class ProximityTracker:
    def __init__(self, serial_port: str, baud: int, camera_id: int,
                 hsv_lower, hsv_upper):
        self.serial_port = serial_port
        self.baud = baud
        self.camera_id = camera_id
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)

        self.ser: serial.Serial | None = None
        self.cap: cv2.VideoCapture | None = None

        self.corners_image: list[tuple[int, int]] = []
        self.homography: np.ndarray | None = None
        self.destination_mm: tuple[float, float] | None = None
        self.destination_px: tuple[int, int] | None = None

        self.last_car_px: tuple[float, float] | None = None
        self.seq = 0

        # CALIBRATING -> SET_DESTINATION -> TRACKING
        self.state = "CALIBRATING"

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.01)
            print(f"Serial connected: {self.serial_port} @ {self.baud}")
        except serial.SerialException as exc:
            print(f"Serial connection failed: {exc}")
            print("Running in vision-only mode (no serial output)")
            self.ser = None

    def open_camera(self):
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"Failed to open camera {self.camera_id}")
            sys.exit(1)
        print(f"Camera {self.camera_id} opened")

    # ------------------------------------------------------------------
    # Mouse callback (calibration + destination)
    # ------------------------------------------------------------------
    def on_mouse(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.state == "CALIBRATING":
            self.corners_image.append((x, y))
            n = len(self.corners_image)
            labels = ["TL", "TR", "BR", "BL"]
            print(f"Corner {n}/4 ({labels[n-1]}): ({x}, {y})")
            if n == 4:
                self._compute_homography()
                self.state = "SET_DESTINATION"
                print("Calibration done. Click inside the maze to set the destination.")

        elif self.state == "SET_DESTINATION":
            pt = np.array([[[x, y]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(pt, self.homography)
            mx, my = float(transformed[0][0][0]), float(transformed[0][0][1])
            self.destination_mm = (mx, my)
            self.destination_px = (x, y)
            self.state = "TRACKING"
            print(f"Destination set at ({mx:.0f}, {my:.0f}) mm")

    def _compute_homography(self):
        src = np.array(self.corners_image, dtype=np.float32)
        dst = np.array([
            [0, 0],
            [MAZE_WIDTH_MM, 0],
            [MAZE_WIDTH_MM, MAZE_HEIGHT_MM],
            [0, MAZE_HEIGHT_MM],
        ], dtype=np.float32)
        self.homography, _ = cv2.findHomography(src, dst)
        print("Homography computed")

    # ------------------------------------------------------------------
    # Arrow detection
    # ------------------------------------------------------------------
    def detect_arrow(self, frame):
        """Return (centroid_px, mask) or (None, mask)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
            return None, mask

        m = cv2.moments(largest)
        if m["m00"] == 0:
            return None, mask

        cx = m["m10"] / m["m00"]
        cy = m["m01"] / m["m00"]
        return (cx, cy), mask

    # ------------------------------------------------------------------
    # Coordinate transform + distance
    # ------------------------------------------------------------------
    def pixel_to_maze_mm(self, px, py):
        pt = np.array([[[px, py]]], dtype=np.float32)
        t = cv2.perspectiveTransform(pt, self.homography)
        return float(t[0][0][0]), float(t[0][0][1])

    def compute_distance(self, car_mm):
        dx = car_mm[0] - self.destination_mm[0]
        dy = car_mm[1] - self.destination_mm[1]
        return float(np.sqrt(dx * dx + dy * dy))

    # ------------------------------------------------------------------
    # Serial output
    # ------------------------------------------------------------------
    def send_motor_params(self, strength, pulse_on, pulse_off, flags):
        if self.ser is None:
            return
        self.seq += 1
        frame = build_frame(self.seq, strength, pulse_on, pulse_off, flags)
        try:
            self.ser.write(frame)
        except serial.SerialException:
            pass

    # ------------------------------------------------------------------
    # Overlay drawing
    # ------------------------------------------------------------------
    def draw_overlay(self, frame, car_px, distance_mm, valid):
        for i, corner in enumerate(self.corners_image):
            cv2.circle(frame, corner, 6, (0, 255, 0), -1)
            cv2.putText(frame, str(i + 1), (corner[0] + 8, corner[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if len(self.corners_image) == 4:
            pts = np.array(self.corners_image, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        if self.destination_px:
            dx, dy = self.destination_px
            cv2.drawMarker(frame, (int(dx), int(dy)),
                           (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        if car_px is not None:
            cx, cy = int(car_px[0]), int(car_px[1])
            cv2.circle(frame, (cx, cy), 8, (255, 0, 0), 2)
            if self.destination_px:
                cv2.line(frame, (cx, cy),
                         (int(self.destination_px[0]),
                          int(self.destination_px[1])),
                         (255, 128, 0), 1)

        if self.state == "CALIBRATING":
            text = f"Click maze corners ({len(self.corners_image)}/4): TL, TR, BR, BL"
        elif self.state == "SET_DESTINATION":
            text = "Click to set destination"
        elif valid and distance_mm is not None:
            text = f"Distance: {distance_mm:.0f} mm"
        else:
            text = "Arrow lost"

        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, "[R] reset cal   [D] new dest   [Q] quit",
                    (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        self.open_camera()
        self.connect_serial()

        cv2.namedWindow("Maze Tracker")
        cv2.setMouseCallback("Maze Tracker", self.on_mouse)

        last_send = 0.0
        send_interval = 0.05  # 20 Hz

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break

                car_px = None
                distance_mm = None
                valid = False

                if self.state == "TRACKING":
                    car_px, _mask = self.detect_arrow(frame)
                    if car_px is not None:
                        car_mm = self.pixel_to_maze_mm(car_px[0], car_px[1])
                        distance_mm = self.compute_distance(car_mm)
                        valid = True
                        self.last_car_px = car_px

                    now = time.monotonic()
                    if now - last_send >= send_interval:
                        last_send = now
                        s, on, off, f = distance_to_motor_params(
                            distance_mm if valid else 0, valid)
                        self.send_motor_params(s, on, off, f)

                self.draw_overlay(frame, car_px, distance_mm, valid)
                cv2.imshow("Maze Tracker", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    self.corners_image.clear()
                    self.homography = None
                    self.destination_mm = None
                    self.destination_px = None
                    self.state = "CALIBRATING"
                    print("Calibration reset")
                elif key == ord("d"):
                    if self.homography is not None:
                        self.destination_mm = None
                        self.destination_px = None
                        self.state = "SET_DESTINATION"
                        print("Click to set new destination")
        except KeyboardInterrupt:
            print("\nInterrupted — shutting down.")
        finally:
            self.send_motor_params(0, 0, 0, 0)
            time.sleep(0.1)
            if self.ser:
                self.ser.close()
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Maze proximity tracker — streams haptic motor params to bridge ESP32")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port for the bridge ESP32")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index")
    parser.add_argument("--hsv-lower", type=int, nargs=3,
                        default=list(DEFAULT_HSV_LOWER),
                        help="HSV lower bound (H S V)")
    parser.add_argument("--hsv-upper", type=int, nargs=3,
                        default=list(DEFAULT_HSV_UPPER),
                        help="HSV upper bound (H S V)")
    args = parser.parse_args()

    tracker = ProximityTracker(
        serial_port=args.port,
        baud=args.baud,
        camera_id=args.camera,
        hsv_lower=tuple(args.hsv_lower),
        hsv_upper=tuple(args.hsv_upper),
    )
    tracker.run()


if __name__ == "__main__":
    main()
