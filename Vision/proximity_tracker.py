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

PROX_STRENGTH = 255
PROX_PULSE_ON_MS = 100
PROX_ARRIVAL_PULSE_ON_MS = 500

# (upper_bound_mm, pulse_off_ms) — 200 mm buckets, closer = faster cadence.
PROX_DISTANCE_BUCKETS = (
    (200, 100),     # 50-200 mm     ~5.0 Hz
    (400, 220),     # 200-400 mm    ~3.1 Hz
    (600, 400),     # 400-600 mm    ~2.0 Hz
    (800, 700),     # 600-800 mm    ~1.25 Hz
    (1000, 1150),   # 800-1000 mm   ~0.8 Hz
    (1200, 1900),   # 1000-1200 mm  ~0.5 Hz
)

# ---------------------------------------------------------------------------
# Arrow detection defaults (bright saturated colour on neutral roof)
# ---------------------------------------------------------------------------
DEFAULT_HSV_LOWER = (5, 150, 100)
DEFAULT_HSV_UPPER = (25, 255, 255)
MIN_CONTOUR_AREA = 200

# Interactive colour-picker defaults
COLOR_PATCH_RADIUS = 5            # 11x11 patch around the click
DEFAULT_H_TOL = 10
DEFAULT_S_TOL = 60
DEFAULT_V_TOL = 60
MAX_H_TOL = 90
MAX_SV_TOL = 127
MASK_WINDOW_NAME = "Mask"


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
        return PROX_STRENGTH, PROX_ARRIVAL_PULSE_ON_MS, 0, FLAG_VALID

    for upper_mm, pulse_off in PROX_DISTANCE_BUCKETS:
        if d <= upper_mm:
            return PROX_STRENGTH, PROX_PULSE_ON_MS, pulse_off, FLAG_VALID

    return PROX_STRENGTH, PROX_PULSE_ON_MS, PROX_DISTANCE_BUCKETS[-1][1], FLAG_VALID


class ProximityTracker:
    def __init__(self, serial_port: str, baud: int, camera_id: int,
                 hsv_lower, hsv_upper, h_tol: int, s_tol: int, v_tol: int,
                 skip_color_pick: bool):
        self.serial_port = serial_port
        self.baud = baud
        self.camera_id = camera_id
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.hsv_ranges: list[tuple[np.ndarray, np.ndarray]] = [
            (self.hsv_lower, self.hsv_upper)
        ]

        self.h_tol = int(h_tol)
        self.s_tol = int(s_tol)
        self.v_tol = int(v_tol)
        self.sampled_hsv: tuple[int, int, int] | None = None
        self.skip_color_pick = skip_color_pick

        self.ser: serial.Serial | None = None
        self.cap: cv2.VideoCapture | None = None
        self._last_frame: np.ndarray | None = None

        self.corners_image: list[tuple[int, int]] = []
        self.homography: np.ndarray | None = None
        self.destination_mm: tuple[float, float] | None = None
        self.destination_px: tuple[int, int] | None = None

        self.last_car_px: tuple[float, float] | None = None
        self.seq = 0

        # CALIBRATING -> PICK_COLOR -> SET_DESTINATION -> TRACKING
        # (PICK_COLOR is skipped when explicit --hsv-lower/--hsv-upper are passed.)
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
                if self.skip_color_pick:
                    self.state = "SET_DESTINATION"
                    print("Calibration done. Click inside the maze to set the destination.")
                else:
                    self.state = "PICK_COLOR"
                    print("Calibration done. Click the car to sample its colour.")

        elif self.state == "PICK_COLOR":
            self.sample_color_at(self._last_frame, x, y)
            self.state = "SET_DESTINATION"
            print("Click inside the maze to set the destination.")

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
    # Interactive colour sampling
    # ------------------------------------------------------------------
    def sample_color_at(self, frame, x: int, y: int) -> None:
        """Sample HSV around (x, y) and update detection bounds."""
        if frame is None:
            print("No frame available for colour sampling.")
            return

        h, w = frame.shape[:2]
        r = COLOR_PATCH_RADIUS
        x0 = max(0, x - r)
        x1 = min(w, x + r + 1)
        y0 = max(0, y - r)
        y1 = min(h, y + r + 1)
        patch = frame[y0:y1, x0:x1]
        if patch.size == 0:
            print("Sample patch is empty (clicked outside frame).")
            return

        hsv_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        med = np.median(hsv_patch.reshape(-1, 3), axis=0)
        self.sampled_hsv = (int(med[0]), int(med[1]), int(med[2]))
        self._recompute_hsv_bounds()
        print(f"Sampled HSV ~ {self.sampled_hsv}  ranges={self._format_ranges()}")

    def _recompute_hsv_bounds(self) -> None:
        """Rebuild self.hsv_ranges from the sampled centre + tolerances."""
        if self.sampled_hsv is None:
            return

        h, s, v = self.sampled_hsv
        s_lo = max(0, s - self.s_tol)
        s_hi = min(255, s + self.s_tol)
        v_lo = max(0, v - self.v_tol)
        v_hi = min(255, v + self.v_tol)

        h_lo_raw = h - self.h_tol
        h_hi_raw = h + self.h_tol

        ranges: list[tuple[np.ndarray, np.ndarray]] = []
        if h_lo_raw < 0:
            # Wrap: [0, h_hi] and [180 + h_lo, 179]
            ranges.append((
                np.array([0, s_lo, v_lo], dtype=np.uint8),
                np.array([h_hi_raw, s_hi, v_hi], dtype=np.uint8),
            ))
            ranges.append((
                np.array([180 + h_lo_raw, s_lo, v_lo], dtype=np.uint8),
                np.array([179, s_hi, v_hi], dtype=np.uint8),
            ))
        elif h_hi_raw > 179:
            ranges.append((
                np.array([h_lo_raw, s_lo, v_lo], dtype=np.uint8),
                np.array([179, s_hi, v_hi], dtype=np.uint8),
            ))
            ranges.append((
                np.array([0, s_lo, v_lo], dtype=np.uint8),
                np.array([h_hi_raw - 180, s_hi, v_hi], dtype=np.uint8),
            ))
        else:
            ranges.append((
                np.array([h_lo_raw, s_lo, v_lo], dtype=np.uint8),
                np.array([h_hi_raw, s_hi, v_hi], dtype=np.uint8),
            ))

        self.hsv_ranges = ranges
        self.hsv_lower, self.hsv_upper = ranges[0]

    def _format_ranges(self) -> str:
        return ", ".join(
            f"[{tuple(int(x) for x in lo)}-{tuple(int(x) for x in hi)}]"
            for lo, hi in self.hsv_ranges
        )

    # ------------------------------------------------------------------
    # Arrow detection
    # ------------------------------------------------------------------
    def detect_arrow(self, frame):
        """Return (centroid_px, mask) or (None, mask)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = None
        for lo, hi in self.hsv_ranges:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        if mask is None:
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)

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
        elif self.state == "PICK_COLOR":
            text = "Click the car to sample its colour"
        elif self.state == "SET_DESTINATION":
            text = "Click to set destination"
        elif valid and distance_mm is not None:
            text = f"Distance: {distance_mm:.0f} mm"
        else:
            text = "Arrow lost"

        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame,
                    "[R] reset cal   [C] re-pick colour   [D] new dest   [Q] quit",
                    (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # ------------------------------------------------------------------
    # Trackbar callbacks
    # ------------------------------------------------------------------
    def _on_h_tol(self, value: int) -> None:
        self.h_tol = int(value)
        self._recompute_hsv_bounds()

    def _on_s_tol(self, value: int) -> None:
        self.s_tol = int(value)
        self._recompute_hsv_bounds()

    def _on_v_tol(self, value: int) -> None:
        self.v_tol = int(value)
        self._recompute_hsv_bounds()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        self.open_camera()
        self.connect_serial()

        # Create the Mask window (with trackbars) FIRST so that Maze Tracker,
        # created after, comes to the front of the cocoa window stack on macOS.
        cv2.namedWindow(MASK_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("H tol", MASK_WINDOW_NAME, self.h_tol, MAX_H_TOL,
                           self._on_h_tol)
        cv2.createTrackbar("S tol", MASK_WINDOW_NAME, self.s_tol, MAX_SV_TOL,
                           self._on_s_tol)
        cv2.createTrackbar("V tol", MASK_WINDOW_NAME, self.v_tol, MAX_SV_TOL,
                           self._on_v_tol)

        cv2.namedWindow("Maze Tracker")
        cv2.setMouseCallback("Maze Tracker", self.on_mouse)

        cv2.moveWindow("Maze Tracker", 50, 50)
        cv2.moveWindow(MASK_WINDOW_NAME, 800, 50)

        # Prime both windows so cocoa actually renders them before the first
        # cap.read() (works around AUTOSIZE+trackbar quirks).
        placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.imshow("Maze Tracker", placeholder)
        cv2.imshow(MASK_WINDOW_NAME, placeholder[:, :, 0])
        cv2.waitKey(1)

        last_send = 0.0
        send_interval = 0.05  # 20 Hz

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                self._last_frame = frame

                car_px = None
                distance_mm = None
                valid = False
                mask = None

                if self.state == "TRACKING":
                    car_px, mask = self.detect_arrow(frame)
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
                elif self.sampled_hsv is not None:
                    # Preview the mask live while the user adjusts tolerances.
                    _, mask = self.detect_arrow(frame)

                if mask is None:
                    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                cv2.imshow(MASK_WINDOW_NAME, mask)

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
                    self.sampled_hsv = None
                    self.state = "CALIBRATING"
                    print("Calibration reset")
                elif key == ord("d"):
                    if self.homography is not None:
                        self.destination_mm = None
                        self.destination_px = None
                        self.state = "SET_DESTINATION"
                        print("Click to set new destination")
                elif key == ord("c"):
                    if self.homography is not None:
                        self.state = "PICK_COLOR"
                        print("Click the car to sample its colour.")
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
    parser.add_argument("--port", default="/dev/cu.usbserial-10",
                        help="Serial port for the bridge ESP32")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index")
    parser.add_argument("--hsv-lower", type=int, nargs=3, default=None,
                        help="HSV lower bound (H S V) — skips interactive pick")
    parser.add_argument("--hsv-upper", type=int, nargs=3, default=None,
                        help="HSV upper bound (H S V) — skips interactive pick")
    parser.add_argument("--h-tol", type=int, default=DEFAULT_H_TOL,
                        help="Initial hue tolerance for interactive picker")
    parser.add_argument("--s-tol", type=int, default=DEFAULT_S_TOL,
                        help="Initial saturation tolerance for interactive picker")
    parser.add_argument("--v-tol", type=int, default=DEFAULT_V_TOL,
                        help="Initial value tolerance for interactive picker")
    args = parser.parse_args()

    explicit_hsv = args.hsv_lower is not None or args.hsv_upper is not None
    hsv_lower = tuple(args.hsv_lower) if args.hsv_lower is not None else DEFAULT_HSV_LOWER
    hsv_upper = tuple(args.hsv_upper) if args.hsv_upper is not None else DEFAULT_HSV_UPPER

    tracker = ProximityTracker(
        serial_port=args.port,
        baud=args.baud,
        camera_id=args.camera,
        hsv_lower=hsv_lower,
        hsv_upper=hsv_upper,
        h_tol=args.h_tol,
        s_tol=args.s_tol,
        v_tol=args.v_tol,
        skip_color_pick=explicit_hsv,
    )
    tracker.run()


if __name__ == "__main__":
    main()
