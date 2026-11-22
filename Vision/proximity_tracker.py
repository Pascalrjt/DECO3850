"""
Overhead camera tracker for the maze game.

Detects a coloured arrow on the RC car roof via HSV thresholding, computes
Euclidean distance to an overseer-set destination through a four-point
homography, maps distance to haptic motor parameters, and streams compact
binary frames to a bridge ESP32 over USB serial.
"""

import argparse
import json
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import serial
from serial.tools import list_ports

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
# The maze is 800 x 600 mm: 7 cells across, 5 cells down. Click the corners with
# the 800 mm side as the top edge.
MAZE_WIDTH_MM = 800
MAZE_HEIGHT_MM = 600
MAZE_COLS = 7
MAZE_ROWS = 5

# Fixed goal: centre of the cell in column 5, row 2 (counted from the top-left),
# as (x, y) in mm from the top-left corner. [D] lets the overseer click another
# spot for one run; [G] returns to this one.
DEFAULT_DESTINATION_MM = (
    (5 - 0.5) * MAZE_WIDTH_MM / MAZE_COLS,    # 514 mm across
    (2 - 0.5) * MAZE_HEIGHT_MM / MAZE_ROWS,   # 180 mm down
)

# ---------------------------------------------------------------------------
# Distance -> motor mapping
# ---------------------------------------------------------------------------
MAX_DISTANCE_MM = 1000.0          # maze diagonal
ARRIVAL_THRESHOLD_MM = 50.0

PROX_STRENGTH = 255
PROX_PULSE_ON_MS = 100
PROX_ARRIVAL_PULSE_ON_MS = 500

# (upper_bound_mm, pulse_off_ms) — one bucket per maze cell (about 115 mm),
# closer = faster cadence. Anything past the last bound uses the slowest cadence.
PROX_DISTANCE_BUCKETS = (
    (115, 100),     # 50-115 mm     ~5.0 Hz
    (230, 220),     # 115-230 mm    ~3.1 Hz
    (345, 400),     # 230-345 mm    ~2.0 Hz
    (460, 700),     # 345-460 mm    ~1.25 Hz
    (575, 1150),    # 460-575 mm    ~0.8 Hz
    (1000, 1900),   # over 575 mm   ~0.5 Hz
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


# ---------------------------------------------------------------------------
# Start-up helpers: find the Bridge and pick the camera without typing flags
# ---------------------------------------------------------------------------
BRIDGE_MARKERS = (b"Forwarded:", b"Proximity bridge ready", b"Bridge MAC:")
BRIDGE_LISTEN_S = 3.5             # the Bridge prints a status line every 2 s
MAX_CAMERA_INDEX = 5              # camera numbers 0..5 are probed at start-up
SETTINGS_PATH = Path(__file__).with_name(".tracker_settings.json")
CAMERA_WINDOW_NAME = "Select Camera"


def load_settings() -> dict:
    try:
        return json.loads(SETTINGS_PATH.read_text())
    except (OSError, ValueError):
        return {}


def save_settings(settings: dict) -> None:
    try:
        SETTINGS_PATH.write_text(json.dumps(settings, indent=2) + "\n")
    except OSError as exc:
        print(f"Could not save settings: {exc}")


def find_bridge(baud: int):
    """Listen on each USB serial port for the Bridge's status output.

    Returns an open serial.Serial for the Bridge, or None when no Bridge answers.
    Opening a port resets the ESP32 on it, so any other board plugged in restarts once.
    """
    candidates = [p.device for p in list_ports.comports()
                  if "usb" in p.device.lower() or p.vid is not None]
    if not candidates:
        print("Looking for the Bridge ESP32... no USB serial ports found.")
        return None

    print("Looking for the Bridge ESP32...")
    for device in sorted(candidates):
        try:
            port = serial.Serial(device, baud, timeout=0.2)
        except (serial.SerialException, OSError) as exc:
            print(f"  {device}   could not open ({exc})")
            continue
        heard = b""
        deadline = time.monotonic() + BRIDGE_LISTEN_S
        found = False
        while time.monotonic() < deadline:
            try:
                heard += port.read(256)
            except (serial.SerialException, OSError):
                break
            if any(marker in heard for marker in BRIDGE_MARKERS):
                found = True
                break
        if found:
            print(f"  {device}   Bridge found")
            port.timeout = 0.01
            return port
        port.close()
        print(f"  {device}   no Bridge status line (another board)")
    return None


def probe_cameras() -> list[int]:
    """Return the camera numbers that open and deliver a picture."""
    found = []
    for index in range(MAX_CAMERA_INDEX + 1):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(index)
        cap.release()
    return found


def choose_camera(preferred: int | None) -> int | None:
    """Show each camera's picture and let the overseer pick one.

    [N] next, [P] previous, [Enter] use it, [Q] quit. Returns None on quit.
    """
    cameras = probe_cameras()
    if not cameras:
        print("No working camera found.")
        return None
    if len(cameras) == 1:
        print(f"Found 1 camera. Using camera {cameras[0]}.")
        return cameras[0]

    position = cameras.index(preferred) if preferred in cameras else 0
    remembered = preferred if preferred in cameras else None
    message = f"Found {len(cameras)} cameras."
    if remembered is not None:
        message += f" Last time you used camera {remembered}."
    print(message)
    print("Pick the camera in the window: [N] next  [P] previous  [Enter] use it")

    cv2.namedWindow(CAMERA_WINDOW_NAME)
    cap = cv2.VideoCapture(cameras[position])
    chosen = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                frame = np.zeros((360, 640, 3), dtype=np.uint8)
            index = cameras[position]
            height, width = frame.shape[:2]
            cv2.rectangle(frame, (0, 0), (width, 40), (0, 0, 0), -1)
            cv2.putText(frame, f"Camera {index} ({position + 1} of {len(cameras)})",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            if index == remembered:
                cv2.putText(frame, "used last time", (width - 170, 27),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 230, 120), 1)
            cv2.rectangle(frame, (0, height - 34), (width, height), (0, 0, 0), -1)
            cv2.putText(frame, "[N] next   [P] previous   [Enter] use this camera   [Q] quit",
                        (10, height - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 215, 255), 1)
            cv2.imshow(CAMERA_WINDOW_NAME, frame)

            key = cv2.waitKey(30) & 0xFF
            if key in (13, 10):
                chosen = index
                break
            if key == ord("q"):
                break
            if key in (ord("n"), ord("p")):
                step = 1 if key == ord("n") else -1
                position = (position + step) % len(cameras)
                cap.release()
                cap = cv2.VideoCapture(cameras[position])
    finally:
        cap.release()
        cv2.destroyWindow(CAMERA_WINDOW_NAME)
        cv2.waitKey(1)
    return chosen


def draw_label(frame, text: str, origin: tuple[int, int], colour, scale: float = 0.5) -> None:
    """Text on a dark backing box, so it reads on any floor colour."""
    (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = origin
    cv2.rectangle(frame, (x - 3, y - height - 4), (x + width + 3, y + baseline + 2), (20, 20, 20), -1)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv2.LINE_AA)


def xor_checksum(data: bytes) -> int:
    chk = 0
    for b in data:
        chk ^= b
    return chk


def build_frame(seq: int, strength: int, pulse_on: int,
                pulse_off: int, flags: int) -> bytes:
    payload = struct.pack(PACKET_FMT, seq, strength, pulse_on, pulse_off, flags)
    return FRAME_MAGIC + payload + bytes([xor_checksum(payload)])


def pulse_rate_label(distance_mm: float) -> str:
    """Human-readable cadence for the overlay."""
    _, on, off, _ = distance_to_motor_params(distance_mm, True)
    if off == 0:
        return "solid on (arrived)"
    return f"{1000.0 / (on + off):.1f} pulses/s"


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
    def __init__(self, serial_port: str | None, baud: int, camera_id: int | None,
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
        self.destination_ring_px: int = 0       # arrival radius drawn around the goal
        self.destination_is_fixed = False       # True while the built-in goal is in use
        # Zone rings around the goal, one per pulse-rate boundary: (bound_mm, segments, label, label_px)
        self.zone_rings: list[tuple[float, list[np.ndarray], str, tuple[int, int] | None]] = []
        self.show_zones = True                  # [Z] toggles the rings

        self.last_car_px: tuple[float, float] | None = None
        self.seq = 0

        # CALIBRATING -> PICK_COLOR -> SET_DESTINATION -> TRACKING
        # (PICK_COLOR is skipped when explicit --hsv-lower/--hsv-upper are passed.)
        self.state = "CALIBRATING"

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def connect_serial(self):
        if self.serial_port is None:
            # No --port given: find the Bridge by its status output
            self.ser = find_bridge(self.baud)
            if self.ser is not None:
                self.serial_port = self.ser.port
                print(f"Serial connected: {self.serial_port} @ {self.baud}")
            else:
                print("No Bridge found. Running in vision-only mode (no serial output)")
            return
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.01)
            print(f"Serial connected: {self.serial_port} @ {self.baud}")
        except serial.SerialException as exc:
            print(f"Serial connection failed: {exc}")
            print("Running in vision-only mode (no serial output)")
            self.ser = None

    def open_camera(self):
        if self.camera_id is None:
            # No --camera given: pick on screen, starting from last time's choice
            settings = load_settings()
            chosen = choose_camera(settings.get("camera"))
            if chosen is None:
                print("No camera selected.")
                if self.ser:
                    self.ser.close()
                sys.exit(1)
            self.camera_id = chosen
            settings["camera"] = chosen
            save_settings(settings)
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
                self.use_fixed_destination()
                if self.skip_color_pick:
                    self.state = "TRACKING"
                    print("Calibration done. Tracking to the fixed goal.")
                else:
                    self.state = "PICK_COLOR"
                    print("Calibration done. Click the car to sample its colour.")

        elif self.state == "PICK_COLOR":
            self.sample_color_at(self._last_frame, x, y)
            if self.destination_mm is not None:
                self.state = "TRACKING"
                print("Colour set. Tracking.")
            else:
                self.state = "SET_DESTINATION"
                print("Click inside the maze to set the destination.")

        elif self.state == "SET_DESTINATION":
            pt = np.array([[[x, y]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(pt, self.homography)
            mx, my = float(transformed[0][0][0]), float(transformed[0][0][1])
            self._set_destination(mx, my, fixed=False)
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

    def maze_mm_to_pixel(self, mx: float, my: float) -> tuple[int, int]:
        pt = np.array([[[mx, my]]], dtype=np.float32)
        t = cv2.perspectiveTransform(pt, np.linalg.inv(self.homography))
        return int(round(float(t[0][0][0]))), int(round(float(t[0][0][1])))

    def _set_destination(self, mx: float, my: float, fixed: bool) -> None:
        self.destination_mm = (mx, my)
        self.destination_px = self.maze_mm_to_pixel(mx, my)
        # Arrival radius in pixels, measured along the maze's x direction
        edge = self.maze_mm_to_pixel(mx + ARRIVAL_THRESHOLD_MM, my)
        self.destination_ring_px = max(4, int(round(float(np.hypot(
            edge[0] - self.destination_px[0], edge[1] - self.destination_px[1])))))
        self.destination_is_fixed = fixed
        self._build_zone_rings()

    def _build_zone_rings(self) -> None:
        """Project each pulse-rate boundary (a circle in maze mm) into the camera picture.

        The circles are drawn through the calibration mapping, so they stay true
        if the camera views the maze at an angle. Parts outside the maze are dropped.
        """
        self.zone_rings = []
        if self.destination_mm is None or self.homography is None:
            return
        gx, gy = self.destination_mm
        inverse = np.linalg.inv(self.homography)
        angles = np.linspace(0.0, 2.0 * np.pi, 145)
        for bound_mm, _pulse_off in PROX_DISTANCE_BUCKETS[:-1]:
            xs = gx + bound_mm * np.cos(angles)
            ys = gy + bound_mm * np.sin(angles)
            inside = (xs >= 0) & (xs <= MAZE_WIDTH_MM) & (ys >= 0) & (ys <= MAZE_HEIGHT_MM)
            pts_mm = np.stack([xs, ys], axis=1).astype(np.float32).reshape(-1, 1, 2)
            pts_px = cv2.perspectiveTransform(pts_mm, inverse).reshape(-1, 2)
            segments, current = [], []
            for point, keep in zip(pts_px, inside):
                if keep:
                    current.append(point)
                elif len(current) > 1:
                    segments.append(np.array(current, np.int32).reshape(-1, 1, 2))
                    current = []
                else:
                    current = []
            if len(current) > 1:
                segments.append(np.array(current, np.int32).reshape(-1, 1, 2))
            # Label each ring with the pulse rate felt just inside it
            label = pulse_rate_label(bound_mm - 1.0).replace(" pulses/s", "/s")
            label_px = None
            if segments:
                longest = max(segments, key=len)
                mid = longest[len(longest) // 2][0]
                label_px = (int(mid[0]) + 4, int(mid[1]) - 4)
            self.zone_rings.append((float(bound_mm), segments, label, label_px))

    def use_fixed_destination(self) -> None:
        """Place the built-in goal. Needs the corner calibration first."""
        if self.homography is None:
            return
        self._set_destination(DEFAULT_DESTINATION_MM[0], DEFAULT_DESTINATION_MM[1], fixed=True)
        print(f"Goal fixed at ({DEFAULT_DESTINATION_MM[0]:.0f}, "
              f"{DEFAULT_DESTINATION_MM[1]:.0f}) mm")

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

        if self.show_zones and self.destination_px and self.zone_rings:
            # The ring that bounds the car's current zone is drawn brighter and thicker
            current_bound = None
            if valid and distance_mm is not None and distance_mm >= ARRIVAL_THRESHOLD_MM:
                current_bound = next((b for b, *_ in self.zone_rings if distance_mm <= b), None)
            for bound_mm, segments, label, label_px in self.zone_rings:
                active = bound_mm == current_bound
                colour = (0, 255, 255) if active else (200, 230, 230)
                # dark underlay first, so the ring shows on light and dark floors alike
                cv2.polylines(frame, segments, False, (30, 30, 30), 5 if active else 3, cv2.LINE_AA)
                cv2.polylines(frame, segments, False, colour, 3 if active else 1, cv2.LINE_AA)
                if label_px is not None:
                    draw_label(frame, label, label_px, colour, 0.45)

        if self.destination_px and self.destination_mm:
            dx, dy = int(self.destination_px[0]), int(self.destination_px[1])
            goal_colour = (0, 215, 255)   # yellow (BGR)
            cv2.circle(frame, (dx, dy), self.destination_ring_px, goal_colour, 2)   # arrival zone
            cv2.circle(frame, (dx, dy), 6, goal_colour, -1)
            cv2.drawMarker(frame, (dx, dy), (0, 0, 0), cv2.MARKER_CROSS, 12, 1)
            label = (f"{'GOAL' if self.destination_is_fixed else 'GOAL (custom)'} "
                     f"({self.destination_mm[0]:.0f}, {self.destination_mm[1]:.0f}) mm")
            pos = (dx + self.destination_ring_px + 6, dy + 5)
            draw_label(frame, label, pos, goal_colour, 0.5)

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
            text = f"Distance: {distance_mm:.0f} mm   {pulse_rate_label(distance_mm)}"
        else:
            text = "Arrow lost"

        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame,
                    "[R] reset cal   [C] re-pick colour   [D] click new goal   [G] fixed goal   [Z] zones   [Q] quit",
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
        self.connect_serial()
        self.open_camera()

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
                    self.destination_is_fixed = False
                    self.zone_rings = []
                    self.sampled_hsv = None
                    self.state = "CALIBRATING"
                    print("Calibration reset")
                elif key == ord("d"):
                    if self.homography is not None:
                        self.destination_mm = None
                        self.destination_px = None
                        self.zone_rings = []
                        self.state = "SET_DESTINATION"
                        print("Click to set new destination")
                elif key == ord("z"):
                    self.show_zones = not self.show_zones
                elif key == ord("g"):
                    if self.homography is not None:
                        self.use_fixed_destination()
                        if self.state == "SET_DESTINATION":
                            self.state = "TRACKING"
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
    parser.add_argument("--port", default=None,
                        help="Serial port for the bridge ESP32 (default: find it automatically)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=None,
                        help="Camera device index (default: pick on screen)")
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
