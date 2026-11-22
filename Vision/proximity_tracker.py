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

import leaderboard as lb
import sounds

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
# Tracking robustness: reject glare and look-alike objects
# ---------------------------------------------------------------------------
# The car tops out near 55 mm/s (72 mm wheels on 28BYJ-48 steppers at 1000 half-steps/s),
# so a detection that needs a much faster move is not the car.
JUMP_SPEED_LIMIT_MM_S = 300.0     # fastest move accepted between detections
JUMP_MARGIN_MM = 40.0             # allowance for detection noise on top of that
RELOCATE_CONFIRM_S = 0.5          # a new, far-away position must hold this long to be accepted
RELOCATE_RADIUS_MM = 40.0         # ...and stay within this radius while it holds
ARROW_MIN_AREA_MM2 = 100.0        # blobs smaller than about 10 x 10 mm are noise
ARROW_MAX_AREA_MM2 = 8000.0       # blobs larger than about 90 x 90 mm are glare or the floor
MAZE_MASK_MARGIN_PX = 20          # look this far outside the clicked corners, no further
POSITION_SMOOTHING = 0.5          # 0 = frozen, 1 = no smoothing
ZONE_HYSTERESIS_MM = 12.0         # the car must pass a ring by this much before the rate changes

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


ZONE_BOUNDS_MM = (ARRIVAL_THRESHOLD_MM,) + tuple(b for b, _ in PROX_DISTANCE_BUCKETS[:-1])


def zone_index(distance_mm: float, current: int | None = None) -> int:
    """Zone number for a distance: 0 = arrived, then one per ring outward.

    With `current` given, a move to the neighbouring zone only counts once the car
    is ZONE_HYSTERESIS_MM past the ring, so the pulse rate does not flicker on a boundary.
    """
    zone = sum(1 for bound in ZONE_BOUNDS_MM if distance_mm >= bound)
    if current is not None and abs(zone - current) == 1:
        boundary = ZONE_BOUNDS_MM[min(zone, current)]
        if abs(distance_mm - boundary) < ZONE_HYSTERESIS_MM:
            return current
    return zone


def zone_motor_params(zone: int):
    """Motor parameters for a zone number."""
    if zone <= 0:
        return PROX_STRENGTH, PROX_ARRIVAL_PULSE_ON_MS, 0, FLAG_VALID
    index = min(zone - 1, len(PROX_DISTANCE_BUCKETS) - 1)
    return PROX_STRENGTH, PROX_PULSE_ON_MS, PROX_DISTANCE_BUCKETS[index][1], FLAG_VALID


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


# ---------------------------------------------------------------------------
# Interface constants
# ---------------------------------------------------------------------------
WINDOW_NAME = "Maze Tracker"
TOP_BAR_H = 62                    # status panel / step banner height
FOOTER_H = 28
BAR_COLOUR = (29, 24, 22)         # the bars sit above and below the picture, never over it
MAX_WORK_WIDTH = 1280             # wider camera pictures are scaled down to fit a laptop screen
ARROW_HOLD_S = 0.5                # keep the last pulse rate this long after losing the arrow
LOUPE_SOURCE = 32                 # pixels around the cursor shown in the zoomed view
LOUPE_SCALE = 4
CORNER_GRAB_RADIUS = 18           # how close a click must be to drag a corner in the grid check
MASK_INSET_W = 170
KEY_ESCAPE = 27
KEY_SPACE = 32
KEY_ENTER = (13, 10)
KEY_BACKSPACE = (8, 127)
CORNER_NAMES = ["top-left", "top-right", "bottom-right", "bottom-left"]

WHITE = (255, 255, 255)
GREY = (214, 206, 201)
GREEN = (102, 255, 0)
OK_GREEN = (135, 231, 126)
YELLOW = (102, 216, 255)
GOAL_YELLOW = (0, 215, 255)
RED = (77, 77, 255)

HELP_LINES = [
    ("N", "new team: type its name"),
    ("Space", "start the run"),
    ("X", "abort the run"),
    ("U", "undo the last result"),
    ("S", "sound on or off"),
    ("Enter", "confirm the current step"),
    ("Backspace", "undo the last corner"),
    ("R", "redo the whole setup"),
    ("C", "re-pick the arrow colour"),
    ("D", "click a different goal"),
    ("G", "back to the fixed goal"),
    ("Z", "show or hide the zone rings"),
    ("M", "show or hide the mask inset"),
    ("H", "show or hide this card"),
    ("Q", "quit"),
]


def shade(frame, x0: int, y0: int, x1: int, y1: int, alpha: float = 0.72) -> None:
    """Darken a rectangle of the picture so text on it stays readable."""
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
    if x1 <= x0 or y1 <= y0:
        return
    region = frame[y0:y1, x0:x1]
    frame[y0:y1, x0:x1] = (region * (1.0 - alpha)).astype(np.uint8)


def put(frame, text: str, origin, colour=WHITE, scale: float = 0.6, thickness: int = 1) -> None:
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thickness, cv2.LINE_AA)


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
        self.bridge_ok = False
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
        self.show_mask = True                   # [M] toggles the mask inset
        self.show_help = False                  # [H] toggles the key card

        self.last_car_px: tuple[float, float] | None = None
        self.last_seen_time = 0.0               # when the arrow was last detected
        self.last_distance_mm: float | None = None
        self.last_params = (0, 0, 0, 0)         # last valid motor parameters sent
        self.pulse_anchor = time.monotonic()    # start of the on-screen pulse cycle
        self.maze_mask: np.ndarray | None = None  # detections outside the maze are ignored
        self.track_mm: tuple[float, float] | None = None   # smoothed car position
        self.zone: int | None = None            # current pulse zone (with hysteresis)
        self.pending_mm: tuple[float, float] | None = None  # far-away candidate being confirmed
        self.pending_since = 0.0
        self.ignored_px: list[tuple[int, int]] = []         # blobs rejected this frame, for the overlay
        self.ignored_total = 0

        # Leaderboard run: idle -> ready (team named) -> countdown -> running -> finished / timeout
        self.board: lb.Leaderboard | None = None
        self.live: lb.LiveState | None = None
        self.run_phase = "idle"
        self.team: str | None = None
        self.run_start = 0.0
        self.countdown_start = 0.0
        self.run_elapsed = 0.0
        self.closest_mm: float | None = None
        self.arrived_since: float | None = None     # when the car entered the goal ring
        self.run_result: dict | None = None         # result card for the run that just ended
        self.name_buffer: str | None = None         # team name being typed; None when not typing
        self.sound: sounds.SoundPlayer | None = None    # run sound effects
        self.last10_started = False
        self.notice = ""                            # one-line message for the overseer
        self.notice_until = 0.0
        self.cursor = (0, 0)
        self.drag_index: int | None = None      # corner being dragged in the grid check
        self.saved_setup: dict | None = None    # setup offered for reuse at start-up
        self.seq = 0

        # RESUME -> (Enter) TRACKING, or (R) the full setup:
        # CALIBRATING -> GRID_CHECK -> PICK_COLOR -> TRACKING
        # PICK_COLOR is skipped when explicit --hsv-lower/--hsv-upper are passed.
        # SET_DESTINATION is entered with [D] to click a different goal.
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
            self.bridge_ok = self.ser is not None
            return
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.01)
            print(f"Serial connected: {self.serial_port} @ {self.baud}")
        except serial.SerialException as exc:
            print(f"Serial connection failed: {exc}")
            print("Running in vision-only mode (no serial output)")
            self.ser = None
        self.bridge_ok = self.ser is not None

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
    # Saved setup (corners + arrow colour), so a normal start is one key press
    # ------------------------------------------------------------------
    def save_setup(self) -> None:
        if len(self.corners_image) != 4 or self._last_frame is None:
            return
        settings = load_settings()
        height, width = self._last_frame.shape[:2]
        settings["setup"] = {
            "camera": self.camera_id,
            "frame_size": [int(width), int(height)],
            "corners": [[int(x), int(y)] for x, y in self.corners_image],
            "sampled_hsv": list(self.sampled_hsv) if self.sampled_hsv else None,
            "tolerances": [self.h_tol, self.s_tol, self.v_tol],
            "saved_at": time.strftime("%d %b, %H:%M"),
        }
        save_settings(settings)

    def offer_saved_setup(self, frame) -> None:
        """Enter RESUME when a saved setup matches this camera and picture size."""
        setup = load_settings().get("setup")
        if not setup:
            return
        height, width = frame.shape[:2]
        corners = setup.get("corners") or []
        if (setup.get("camera") != self.camera_id
                or setup.get("frame_size") != [int(width), int(height)]
                or len(corners) != 4):
            return
        self.saved_setup = setup
        self.corners_image = [(int(x), int(y)) for x, y in corners]
        self._compute_homography(quiet=True)
        self.use_fixed_destination(quiet=True)
        self.state = "RESUME"
        print(f"Saved setup found (from {setup.get('saved_at', 'an earlier run')}). "
              "[Enter] use it, [R] set up again.")

    def accept_saved_setup(self) -> None:
        setup = self.saved_setup or {}
        if not self.skip_color_pick and setup.get("sampled_hsv"):
            self.sampled_hsv = tuple(int(v) for v in setup["sampled_hsv"])
            tolerances = setup.get("tolerances") or [self.h_tol, self.s_tol, self.v_tol]
            self.h_tol, self.s_tol, self.v_tol = (int(v) for v in tolerances)
            self._recompute_hsv_bounds()
            self._sync_trackbars()
        if self.skip_color_pick or self.sampled_hsv is not None:
            self.state = "TRACKING"
            print("Using the saved setup. Tracking.")
        else:
            self.state = "PICK_COLOR"
            print("Using the saved corners. Click the car to sample its colour.")

    def restart_setup(self) -> None:
        self.corners_image.clear()
        self.homography = None
        self.destination_mm = None
        self.destination_px = None
        self.destination_is_fixed = False
        self.zone_rings = []
        self.sampled_hsv = None
        self.saved_setup = None
        self.drag_index = None
        self.reset_track()
        self.state = "CALIBRATING"
        print("Setup restarted. Click the four corner posts.")

    # ------------------------------------------------------------------
    # Mouse callback (corners, corner dragging, colour, custom goal)
    # ------------------------------------------------------------------
    def on_mouse(self, event, x, y, _flags, _param):
        self.cursor = (x, y)

        if event == cv2.EVENT_MOUSEMOVE:
            if self.drag_index is not None and self.state == "GRID_CHECK":
                self.corners_image[self.drag_index] = (x, y)
                self._compute_homography(quiet=True)
                self.use_fixed_destination(quiet=True)
            return

        if event == cv2.EVENT_LBUTTONUP:
            self.drag_index = None
            return

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
                self.state = "GRID_CHECK"
                print("Check the grid: lines should run through the posts. "
                      "[Enter] looks right, drag a corner to adjust.")

        elif self.state == "GRID_CHECK":
            distances = [np.hypot(x - cx, y - cy) for cx, cy in self.corners_image]
            nearest = int(np.argmin(distances))
            if distances[nearest] <= CORNER_GRAB_RADIUS:
                self.drag_index = nearest

        elif self.state == "PICK_COLOR":
            self.sample_color_at(self._last_frame, x, y)
            self._sync_trackbars()
            self.reset_track()
            if self.destination_mm is not None:
                self.state = "TRACKING"
                self.save_setup()
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

    def _on_window_mouse(self, event, x, y, flags, param):
        """Window clicks arrive below the top bar; shift them into picture coordinates."""
        self.on_mouse(event, x, y - TOP_BAR_H, flags, param)

    @staticmethod
    def fit_frame(frame):
        """Scale very wide camera pictures down so the window fits on screen."""
        height, width = frame.shape[:2]
        if width <= MAX_WORK_WIDTH:
            return frame
        scale = MAX_WORK_WIDTH / width
        return cv2.resize(frame, (MAX_WORK_WIDTH, int(round(height * scale))),
                          interpolation=cv2.INTER_AREA)

    def confirm_grid(self) -> None:
        """[Enter] in the grid check: carry on to colour picking or tracking."""
        if self.skip_color_pick or self.sampled_hsv is not None:
            self.state = "TRACKING"
            self.save_setup()
            print("Calibration done. Tracking to the fixed goal.")
        else:
            self.state = "PICK_COLOR"
            print("Calibration done. Click the car to sample its colour.")

    def undo_corner(self) -> None:
        """[Backspace]: remove the last corner clicked."""
        if self.state == "GRID_CHECK":
            self.state = "CALIBRATING"
        if self.state == "CALIBRATING" and self.corners_image:
            self.corners_image.pop()
            self.homography = None
            self.destination_mm = None
            self.destination_px = None
            self.zone_rings = []
            print(f"Corner removed. {len(self.corners_image)}/4 placed.")

    def _compute_homography(self, quiet: bool = False):
        src = np.array(self.corners_image, dtype=np.float32)
        dst = np.array([
            [0, 0],
            [MAZE_WIDTH_MM, 0],
            [MAZE_WIDTH_MM, MAZE_HEIGHT_MM],
            [0, MAZE_HEIGHT_MM],
        ], dtype=np.float32)
        self.homography, _ = cv2.findHomography(src, dst)
        self.maze_mask = None       # rebuilt for the next frame's size
        if not quiet:
            print("Homography computed")

    def _maze_mask_for(self, shape) -> np.ndarray | None:
        """Mask of the calibrated maze area, slightly enlarged, matching the picture size."""
        if len(self.corners_image) != 4:
            return None
        if self.maze_mask is None or self.maze_mask.shape != shape[:2]:
            mask = np.zeros(shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [np.array(self.corners_image, np.int32)], 255)
            size = 2 * MAZE_MASK_MARGIN_PX + 1
            self.maze_mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))
        return self.maze_mask

    def _mm2_per_px2(self, px: float, py: float) -> float:
        """Local scale of the calibration: how many square mm one pixel covers here."""
        ox, oy = self.pixel_to_maze_mm(px, py)
        ax, ay = self.pixel_to_maze_mm(px + 1.0, py)
        bx, by = self.pixel_to_maze_mm(px, py + 1.0)
        return abs((ax - ox) * (by - oy) - (ay - oy) * (bx - ox))

    def reset_track(self) -> None:
        self.track_mm = None
        self.zone = None
        self.pending_mm = None
        self.last_car_px = None
        self.last_params = (0, 0, 0, 0)

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

    def use_fixed_destination(self, quiet: bool = False) -> None:
        """Place the built-in goal. Needs the corner calibration first."""
        if self.homography is None:
            return
        self._set_destination(DEFAULT_DESTINATION_MM[0], DEFAULT_DESTINATION_MM[1], fixed=True)
        if not quiet:
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
    def find_candidates(self, frame):
        """Every blob of the arrow colour inside the maze: ([(centroid_px, area_px)], mask)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = None
        for lo, hi in self.hsv_ranges:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        if mask is None:
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        maze_mask = self._maze_mask_for(frame.shape)
        if maze_mask is not None:
            mask = cv2.bitwise_and(mask, maze_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_CONTOUR_AREA / 4:
                continue
            m = cv2.moments(contour)
            if m["m00"] == 0:
                continue
            candidates.append(((m["m10"] / m["m00"], m["m01"] / m["m00"]), area))
        candidates.sort(key=lambda c: c[1], reverse=True)
        return candidates, mask

    def locate_car(self, frame, now: float):
        """Pick the blob that is the car, using its size and where the car was last seen.

        Returns (car_px or None, mask). Rejected blobs are remembered for the overlay.
        """
        candidates, mask = self.find_candidates(frame)
        self.ignored_px = []
        plausible = []
        for (px, py), area_px in candidates:
            area_mm2 = area_px * self._mm2_per_px2(px, py)
            if ARROW_MIN_AREA_MM2 <= area_mm2 <= ARROW_MAX_AREA_MM2:
                plausible.append(((px, py), self.pixel_to_maze_mm(px, py), area_px))
            else:
                self.ignored_px.append((int(px), int(py)))

        chosen = None
        if not plausible:
            self.pending_mm = None
        elif self.track_mm is None:
            chosen = plausible[0]                       # first sighting: the largest plausible blob
        else:
            # How far the car could have moved since it was last seen
            reach = JUMP_MARGIN_MM + JUMP_SPEED_LIMIT_MM_S * max(0.0, now - self.last_seen_time)
            in_reach = [c for c in plausible
                        if np.hypot(c[1][0] - self.track_mm[0], c[1][1] - self.track_mm[1]) <= reach]
            if in_reach:
                chosen = min(in_reach, key=lambda c: np.hypot(c[1][0] - self.track_mm[0],
                                                              c[1][1] - self.track_mm[1]))
                self.pending_mm = None
            else:
                # Nothing where the car could be. A far-away blob must hold still before it counts.
                far = plausible[0]
                if (self.pending_mm is not None
                        and np.hypot(far[1][0] - self.pending_mm[0],
                                     far[1][1] - self.pending_mm[1]) <= RELOCATE_RADIUS_MM):
                    if now - self.pending_since >= RELOCATE_CONFIRM_S:
                        chosen = far                    # the car really was moved
                        self.track_mm = None            # restart smoothing at the new place
                        self.zone = None
                        self.pending_mm = None
                else:
                    self.pending_mm = far[1]
                    self.pending_since = now

        for candidate in plausible:
            if candidate is not chosen:
                self.ignored_px.append((int(candidate[0][0]), int(candidate[0][1])))
        self.ignored_total += len(self.ignored_px)

        if chosen is None:
            return None, mask

        mx, my = chosen[1]
        if self.track_mm is None:
            self.track_mm = (mx, my)
        else:
            a = POSITION_SMOOTHING
            self.track_mm = (a * mx + (1 - a) * self.track_mm[0], a * my + (1 - a) * self.track_mm[1])
        return self.maze_mm_to_pixel(*self.track_mm), mask

    def detect_arrow(self, frame):
        """Largest blob of the arrow colour, for the colour-picking preview: (centroid_px, mask)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = None
        for lo, hi in self.hsv_ranges:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        if mask is None:
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        maze_mask = self._maze_mask_for(frame.shape)
        if maze_mask is not None:
            mask = cv2.bitwise_and(mask, maze_mask)

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
    # Leaderboard runs
    # ------------------------------------------------------------------
    def attach_leaderboard(self, board, live) -> None:
        self.board = board
        self.live = live

    def say(self, text: str, now: float | None = None, seconds: float = 4.0) -> None:
        """Show a short message to the overseer, in the window and the terminal."""
        print(text)
        self.notice = text
        self.notice_until = (time.monotonic() if now is None else now) + seconds

    def play(self, name: str | None) -> None:
        """Play a run sound, or stop the current one with None."""
        if self.sound is None:
            return
        if name is None:
            self.sound.stop()
        else:
            self.sound.play(name)

    def begin_name_entry(self) -> None:
        if self.board is None:
            self.say("The leaderboard is switched off.")
        elif self.run_phase in ("running", "countdown"):
            self.say("A run is in progress. Press X to abort it first.")
        else:
            self.name_buffer = ""

    def commit_name(self) -> None:
        name = (self.name_buffer or "").strip()
        if not name:
            self.say("Type a team name first, or press Esc to cancel.")
            return
        if self.board.name_taken(name):
            self.say(f"{name} is already on the board. Pick another name.")
            return
        self.team = name
        self.name_buffer = None
        self.run_phase = "ready"
        self.run_result = None
        self.say(f"{name} is ready. Press Space to start the clock.")

    def start_run(self, now: float) -> None:
        if self.run_phase != "ready":
            self.say("Press N and enter a team name first.", now)
        elif self.state != "TRACKING":
            self.say("Finish the setup before starting a run.", now)
        else:
            self.run_phase = "countdown"
            self.last10_started = False
            self.play("start")                  # three notes, then the "go" chord
            self.countdown_start = now
            self.run_elapsed = 0.0
            self.closest_mm = None
            self.arrived_since = None
            self.say(f"Countdown for {self.team}. Press X to cancel.", now, lb.COUNTDOWN_S)

    def abort_run(self, now: float) -> None:
        if self.run_phase in ("running", "countdown"):
            self.play(None)
            self.run_phase = "ready"
            self.run_elapsed = 0.0
            self.say(f"Run aborted. {self.team} can start again with Space.", now)

    def undo_last_result(self, now: float) -> None:
        if self.board is None or self.run_phase in ("running", "countdown"):
            return
        entry = self.board.undo_last()
        if entry is None:
            self.say("There is no result to undo.", now)
            return
        if self.run_result and self.run_result.get("id") == entry["id"]:
            self.run_result = None
            self.run_phase = "idle"
            self.team = None
        self.say(f"Removed the result for {entry['team']}. Restore it from the manage page if needed.", now)

    def _finish_run(self, finished: bool, now: float) -> None:
        distance = None if finished else self.last_distance_mm
        entry = self.board.add(self.team, finished, self.run_elapsed, distance)
        self.run_result = dict(self.board.describe(entry["id"]), id=entry["id"])
        self.run_phase = "finished" if finished else "timeout"
        self.play("goal" if finished else "timesup")
        if finished:
            self.say(f"{self.team} reached the goal in {lb.format_time(self.run_elapsed)}: "
                     f"{self.run_result.get('place_text', '')}.", now, 8.0)
        else:
            self.say(f"Time's up for {self.team}: {self.run_result.get('result', '')}, "
                     f"{self.run_result.get('place_text', '')}.", now, 8.0)

    def countdown_number(self, now: float) -> int:
        """3, 2 or 1 during the countdown; 0 otherwise."""
        if self.run_phase != "countdown":
            return 0
        return max(1, int(np.ceil(lb.COUNTDOWN_S - (now - self.countdown_start))))

    def update_run(self, distance_mm, valid: bool, now: float) -> None:
        """Advance the countdown and the run clock, and decide whether the run has ended."""
        if self.run_phase == "countdown" and now - self.countdown_start >= lb.COUNTDOWN_S:
            self.run_phase = "running"
            self.run_start = now
            self.say(f"Go! Run started for {self.team}.", now, 2.0)
        if self.run_phase == "running":
            self.run_elapsed = min(now - self.run_start, lb.RUN_LIMIT_S)
            if not self.last10_started and lb.RUN_LIMIT_S - self.run_elapsed <= sounds.LAST_SECONDS:
                self.last10_started = True
                self.play("last10")             # suspense groove for the final seconds
            if valid and distance_mm is not None:
                self.closest_mm = distance_mm if self.closest_mm is None else min(self.closest_mm, distance_mm)
                # Finish: the car has stayed inside the goal ring for a moment
                if distance_mm < ARRIVAL_THRESHOLD_MM:
                    if self.arrived_since is None:
                        self.arrived_since = now
                    elif now - self.arrived_since >= lb.ARRIVAL_DWELL_S:
                        self._finish_run(True, now)
                else:
                    self.arrived_since = None
            if self.run_phase == "running" and now - self.run_start >= lb.RUN_LIMIT_S:
                self._finish_run(False, now)
        if self.live is not None:
            self.live.set(phase=self.run_phase, team=self.team, elapsed_s=self.run_elapsed,
                          remaining_s=max(0.0, lb.RUN_LIMIT_S - self.run_elapsed),
                          distance_mm=distance_mm if valid else self.last_distance_mm,
                          closest_mm=self.closest_mm, result=self.run_result,
                          countdown=self.countdown_number(now))

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
            self.bridge_ok = True
        except (serial.SerialException, OSError):
            self.bridge_ok = False

    def motor_params_for(self, distance_mm, valid: bool, now: float):
        """Motor parameters for this frame, holding the last rate briefly when the arrow is lost."""
        if valid:
            self.zone = zone_index(min(distance_mm, MAX_DISTANCE_MM), self.zone)
            params = zone_motor_params(self.zone)
            if params[1:3] != self.last_params[1:3]:
                self.pulse_anchor = now          # restart the on-screen pulse with the new rate
            self.last_params = params
            return params
        if self.last_params[3] and now - self.last_seen_time <= ARROW_HOLD_S:
            return self.last_params
        self.last_params = (0, 0, 0, 0)
        return self.last_params

    def pulse_is_on(self, params, now: float) -> bool:
        """Whether the motor is in the 'on' part of its pulse right now (for the pulse light)."""
        _strength, on_ms, off_ms, flags = params
        if not flags:
            return False
        if off_ms == 0:
            return True
        phase_ms = ((now - self.pulse_anchor) * 1000.0) % (on_ms + off_ms)
        return phase_ms < on_ms

    # ------------------------------------------------------------------
    # Overlay drawing
    # ------------------------------------------------------------------
    def draw_overlay(self, frame, car_px, distance_mm, valid, mask=None, params=(0, 0, 0, 0)):
        height, width = frame.shape[:2]
        clean = frame.copy() if self.state == "CALIBRATING" else None   # source for the zoomed view
        now = time.monotonic()
        tracking = self.state == "TRACKING"
        setting_up = self.state in ("CALIBRATING", "GRID_CHECK", "RESUME")

        # --- maze outline, grid and corners ---
        if len(self.corners_image) == 4:
            pts = np.array(self.corners_image, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, GREEN, 2, cv2.LINE_AA)
            if self.state in ("GRID_CHECK", "RESUME") and self.homography is not None:
                self._draw_grid(frame)
        if setting_up or len(self.corners_image) < 4:
            for i, corner in enumerate(self.corners_image):
                cv2.circle(frame, corner, 6, GREEN, -1, cv2.LINE_AA)
                put(frame, str(i + 1), (corner[0] + 9, corner[1] - 9), GREEN, 0.55)

        # --- zone rings ---
        if self.show_zones and self.destination_px and self.zone_rings and not setting_up:
            current_bound = None
            if valid and self.zone is not None and 1 <= self.zone <= len(self.zone_rings):
                current_bound = self.zone_rings[self.zone - 1][0]
            elif valid and self.zone is None and distance_mm is not None and distance_mm >= ARRIVAL_THRESHOLD_MM:
                current_bound = next((b for b, *_ in self.zone_rings if distance_mm <= b), None)
            for bound_mm, segments, label, label_px in self.zone_rings:
                active = bound_mm == current_bound
                colour = (0, 255, 255) if active else (200, 230, 230)
                # dark underlay first, so the ring shows on light and dark floors alike
                cv2.polylines(frame, segments, False, (30, 30, 30), 5 if active else 3, cv2.LINE_AA)
                cv2.polylines(frame, segments, False, colour, 3 if active else 1, cv2.LINE_AA)
                if label_px is not None:
                    draw_label(frame, label, label_px, colour, 0.45)

        # --- goal ---
        if self.destination_px and self.destination_mm:
            dx, dy = int(self.destination_px[0]), int(self.destination_px[1])
            cv2.circle(frame, (dx, dy), self.destination_ring_px, GOAL_YELLOW, 2, cv2.LINE_AA)   # arrival zone
            cv2.circle(frame, (dx, dy), 6, GOAL_YELLOW, -1, cv2.LINE_AA)
            cv2.drawMarker(frame, (dx, dy), (0, 0, 0), cv2.MARKER_CROSS, 12, 1)
            label = (f"{'GOAL' if self.destination_is_fixed else 'GOAL (custom)'} "
                     f"({self.destination_mm[0]:.0f}, {self.destination_mm[1]:.0f}) mm")
            draw_label(frame, label, (dx + self.destination_ring_px + 6, dy + 5), GOAL_YELLOW, 0.5)

        # --- blobs of the right colour that were rejected (glare, look-alikes) ---
        if tracking:
            for ix, iy in self.ignored_px[:12]:
                cv2.drawMarker(frame, (ix, iy), (30, 30, 30), cv2.MARKER_TILTED_CROSS, 16, 4)
                cv2.drawMarker(frame, (ix, iy), (200, 200, 200), cv2.MARKER_TILTED_CROSS, 14, 2)

        # --- car, or where it was last seen ---
        if car_px is not None:
            cx, cy = int(car_px[0]), int(car_px[1])
            if self.destination_px:
                cv2.line(frame, (cx, cy), (int(self.destination_px[0]), int(self.destination_px[1])),
                         (255, 163, 77), 1, cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 12, (255, 163, 77), 2, cv2.LINE_AA)
        elif tracking and self.last_car_px is not None:
            lx, ly = int(self.last_car_px[0]), int(self.last_car_px[1])
            cv2.circle(frame, (lx, ly), 16, RED, 2, cv2.LINE_AA)
            draw_label(frame, "last seen here", (lx - 48, ly + 34), WHITE, 0.42)

        # --- things drawn inside the picture ---
        if tracking and not valid:
            cv2.rectangle(frame, (2, 2), (width - 3, height - 3), RED, 5)
        if clean is not None:
            self._draw_loupe(frame, clean)       # zoomed view for precise corner clicks
        if self.show_mask and mask is not None and (tracking or self.state == "PICK_COLOR"
                                                     or self.sampled_hsv is not None):
            self._draw_mask_inset(frame, mask)
        if self.state == "RESUME":
            self._draw_resume_card(frame)
        if self.run_phase == "countdown":
            digit = str(self.countdown_number(now))
            (tw, th), _ = cv2.getTextSize(digit, cv2.FONT_HERSHEY_SIMPLEX, 6.0, 12)
            origin = ((width - tw) // 2, (height + th) // 2)
            cv2.putText(frame, digit, origin, cv2.FONT_HERSHEY_SIMPLEX, 6.0, (20, 20, 20), 22, cv2.LINE_AA)
            cv2.putText(frame, digit, origin, cv2.FONT_HERSHEY_SIMPLEX, 6.0, GOAL_YELLOW, 12, cv2.LINE_AA)

        # --- bars above and below the picture, so nothing covers the maze ---
        canvas = cv2.copyMakeBorder(frame, TOP_BAR_H, FOOTER_H, 0, 0,
                                    cv2.BORDER_CONSTANT, value=BAR_COLOUR)
        if tracking:
            self._draw_status_panel(canvas, distance_mm, valid, params, now)
        else:
            self._draw_step_banner(canvas)
        put(canvas, self._footer_text(), (10, canvas.shape[0] - 9), YELLOW, 0.48)
        if self.notice and now < self.notice_until:
            draw_label(canvas, self.notice, (12, TOP_BAR_H + 22), WHITE, 0.5)
        if self.name_buffer is not None:
            self._draw_name_card(canvas, now)
        if self.show_help:
            self._draw_help_card(canvas)
        return canvas

    def _draw_name_card(self, frame, now: float) -> None:
        height, width = frame.shape[:2]
        card_w, card_h = 400, 140
        x0, y0 = (width - card_w) // 2, (height - card_h) // 2
        shade(frame, x0, y0, x0 + card_w, y0 + card_h, 0.86)
        cv2.rectangle(frame, (x0, y0), (x0 + card_w, y0 + card_h), GREY, 1)
        put(frame, "Team name", (x0 + 20, y0 + 32), WHITE, 0.65, 2)
        cv2.rectangle(frame, (x0 + 20, y0 + 46), (x0 + card_w - 20, y0 + 84), (48, 35, 29), -1)
        cv2.rectangle(frame, (x0 + 20, y0 + 46), (x0 + card_w - 20, y0 + 84), YELLOW, 1)
        cursor = "|" if int(now * 2) % 2 == 0 else ""
        put(frame, (self.name_buffer or "") + cursor, (x0 + 30, y0 + 72), WHITE, 0.65)
        put(frame, "[Enter] save name", (x0 + 20, y0 + 118), OK_GREEN, 0.5)
        put(frame, "[Esc] cancel", (x0 + 220, y0 + 118), YELLOW, 0.5)

    def _draw_grid(self, frame) -> None:
        """Cell grid from the calibration: lines should run through the posts."""
        for col in range(1, MAZE_COLS):
            x = col * MAZE_WIDTH_MM / MAZE_COLS
            cv2.line(frame, self.maze_mm_to_pixel(x, 0), self.maze_mm_to_pixel(x, MAZE_HEIGHT_MM),
                     GREEN, 1, cv2.LINE_AA)
        for row in range(1, MAZE_ROWS):
            y = row * MAZE_HEIGHT_MM / MAZE_ROWS
            cv2.line(frame, self.maze_mm_to_pixel(0, y), self.maze_mm_to_pixel(MAZE_WIDTH_MM, y),
                     GREEN, 1, cv2.LINE_AA)

    def _step_text(self) -> tuple[str, str]:
        total = 2 if self.skip_color_pick else 3
        if self.state == "RESUME":
            return "Start", "Check the green outline still fits the maze"
        if self.state == "CALIBRATING":
            n = len(self.corners_image)
            return (f"Step 1 of {total}",
                    f"Click corner {n + 1}: {CORNER_NAMES[min(n, 3)]} post, at the floor")
        if self.state == "GRID_CHECK":
            return f"Step 2 of {total}", "Grid lines should run through the posts"
        if self.state == "PICK_COLOR":
            return f"Step 3 of {total}", "Click the arrow on the car to sample its colour"
        if self.state == "SET_DESTINATION":
            return "Goal", "Click inside the maze to place the goal"
        return "", ""

    def _draw_step_banner(self, frame) -> None:
        step, text = self._step_text()
        (tw, _th), _ = cv2.getTextSize(step, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (10, 16), (26 + tw, 46), YELLOW, -1)
        put(frame, step, (18, 37), (30, 28, 28), 0.5)
        put(frame, text, (40 + tw, 38), WHITE, 0.62)

    def _draw_status_panel(self, frame, distance_mm, valid, params, now) -> None:
        width = frame.shape[1]
        held = (not valid) and bool(params[3])
        if valid and distance_mm is not None:
            put(frame, f"{distance_mm:.0f} mm", (14, 28), WHITE, 0.85, 2)
            put(frame, "to goal, straight line", (14, 51), GREY, 0.45)
        else:
            put(frame, "Arrow lost", (14, 28), (138, 138, 255), 0.85, 2)
            put(frame, "holding the last rate" if held else "motor paused", (14, 51), GREY, 0.45)

        # pulse light: lit while the motor is in the 'on' part of its cycle
        light_x = min(230, width // 3)
        if params[3]:
            if self.pulse_is_on(params, now):
                cv2.circle(frame, (light_x, 31), 15, GOAL_YELLOW, -1, cv2.LINE_AA)
            cv2.circle(frame, (light_x, 31), 19, GOAL_YELLOW, 2, cv2.LINE_AA)
            rate = ("solid on (arrived)" if params[2] == 0
                    else f"{1000.0 / (params[1] + params[2]):.1f} pulses/s")
            put(frame, rate, (light_x + 30, 28), WHITE, 0.6)
            put(frame, "blinks with the motor", (light_x + 30, 51), GREY, 0.42)
        else:
            cv2.circle(frame, (light_x, 31), 17, (128, 114, 107), 2, cv2.LINE_AA)
            put(frame, "no pulses", (light_x + 30, 36), (175, 163, 154), 0.6)

        # leaderboard run: team and clock, between the pulse light and the status lights
        run_x = light_x + 250
        if self.board is not None and width - 200 - run_x >= 190:
            if self.run_phase == "running":
                put(frame, lb.format_time(self.run_elapsed), (run_x, 34), WHITE, 0.95, 2)
                put(frame, f"{self.team}  ({lb.format_time(lb.RUN_LIMIT_S - self.run_elapsed + 0.999)} left)",
                    (run_x, 53), GOAL_YELLOW, 0.42)
            elif self.run_phase == "countdown":
                put(frame, f"Starting in {self.countdown_number(now)}", (run_x, 30), WHITE, 0.75, 2)
                put(frame, self.team or "", (run_x, 53), GOAL_YELLOW, 0.42)
            elif self.run_phase == "ready":
                put(frame, self.team or "", (run_x, 28), GOAL_YELLOW, 0.6)
                put(frame, "ready: Space starts the clock", (run_x, 51), GREY, 0.42)
            elif self.run_phase in ("finished", "timeout") and self.run_result:
                heading = "Goal reached" if self.run_phase == "finished" else "Time's up"
                put(frame, f"{heading}: {self.run_result.get('result', '')}", (run_x, 28), OK_GREEN, 0.55)
                put(frame, f"{self.team}, {self.run_result.get('place_text', '')}", (run_x, 51), GREY, 0.42)
            else:
                put(frame, "No team", (run_x, 28), GREY, 0.6)
                put(frame, "N enters the next team", (run_x, 51), GREY, 0.42)

        # connection lights
        status_x = max(light_x + 230, width - 200)
        bridge_text = ("Bridge connected" if self.bridge_ok
                       else "Bridge not found" if self.ser is None else "Bridge write failed")
        cv2.circle(frame, (status_x, 21), 5, OK_GREEN if self.bridge_ok else RED, -1, cv2.LINE_AA)
        put(frame, bridge_text, (status_x + 12, 26), WHITE, 0.45)
        cv2.circle(frame, (status_x, 45), 5, OK_GREEN if valid else RED, -1, cv2.LINE_AA)
        put(frame, "Arrow found" if valid else "Arrow lost", (status_x + 12, 50), WHITE, 0.45)

    def _draw_loupe(self, frame, clean) -> None:
        """Magnified view around the cursor, parked in the corner the cursor is not in."""
        height, width = frame.shape[:2]
        x, y = self.cursor
        half = LOUPE_SOURCE // 2
        x0, y0 = int(np.clip(x - half, 0, max(0, width - LOUPE_SOURCE))), \
            int(np.clip(y - half, 0, max(0, height - LOUPE_SOURCE)))
        patch = clean[y0:y0 + LOUPE_SOURCE, x0:x0 + LOUPE_SOURCE]
        if patch.shape[0] < 2 or patch.shape[1] < 2:
            return
        size = LOUPE_SOURCE * LOUPE_SCALE
        zoom = cv2.resize(patch, (size, size), interpolation=cv2.INTER_NEAREST)
        # crosshair at the cursor's position inside the zoomed patch
        cxz, cyz = (x - x0) * LOUPE_SCALE, (y - y0) * LOUPE_SCALE
        cv2.line(zoom, (0, cyz), (size - 1, cyz), WHITE, 1)
        cv2.line(zoom, (cxz, 0), (cxz, size - 1), WHITE, 1)
        left = x > width // 2                       # cursor on the right -> loupe on the left
        lx = 14 if left else width - size - 14
        ly = height - size - 26
        if ly < 4:
            return
        frame[ly:ly + size, lx:lx + size] = zoom
        cv2.rectangle(frame, (lx - 1, ly - 1), (lx + size, ly + size), WHITE, 2)
        draw_label(frame, f"{LOUPE_SCALE}x zoom", (lx + 4, ly + size + 17), WHITE, 0.42)

    def _draw_mask_inset(self, frame, mask) -> None:
        height, width = frame.shape[:2]
        inset_h = int(MASK_INSET_W * mask.shape[0] / mask.shape[1])
        small = cv2.cvtColor(cv2.resize(mask, (MASK_INSET_W, inset_h)), cv2.COLOR_GRAY2BGR)
        x0 = width - MASK_INSET_W - 12
        y0 = height - inset_h - 12
        if y0 < 4:
            return
        frame[y0:y0 + inset_h, x0:x0 + MASK_INSET_W] = small
        cv2.rectangle(frame, (x0 - 1, y0 - 1), (x0 + MASK_INSET_W, y0 + inset_h), WHITE, 1)
        draw_label(frame, "detection mask", (x0 + 4, y0 + 15), GREY, 0.4)

    def _footer_text(self) -> str:
        if self.state == "RESUME":
            return "[Enter] use the saved setup    [R] set up again    [H] help"
        if self.state == "CALIBRATING":
            return "[Backspace] undo last corner    [R] start over    [H] help"
        if self.state == "GRID_CHECK":
            return "[Enter] looks right    [Backspace] redo corners    drag a corner to adjust"
        if self.state == "PICK_COLOR":
            return "Click the arrow, then tune with the tolerance sliders    [H] help"
        if self.state == "SET_DESTINATION":
            return "Click the new goal    [G] back to the fixed goal    [H] help"
        if self.name_buffer is not None:
            return "Type the team name    [Enter] save    [Esc] cancel"
        if self.board is not None:
            if self.run_phase == "countdown":
                return "[X] cancel the start    [H] help"
            if self.run_phase == "running":
                return "[X] abort run    [H] help    [Z] zones    [M] mask"
            if self.run_phase == "ready":
                return "[Space] start run    [N] change team    [H] help    [Q] quit"
            if self.run_phase in ("finished", "timeout"):
                return "[N] next team    [U] undo this result    [H] help    [Q] quit"
            return "[N] new team    [U] undo last result    [S] sound    [H] help    [Z] zones    [M] mask    [Q] quit"
        return "[H] help    [Z] zones    [M] mask    [D] new goal    [C] colour    [R] redo setup    [Q] quit"

    def _draw_resume_card(self, frame) -> None:
        height, width = frame.shape[:2]
        card_w, card_h = 380, 130
        x0, y0 = (width - card_w) // 2, (height - card_h) // 2
        shade(frame, x0, y0, x0 + card_w, y0 + card_h, 0.82)
        cv2.rectangle(frame, (x0, y0), (x0 + card_w, y0 + card_h), GREY, 1)
        saved_at = (self.saved_setup or {}).get("saved_at", "an earlier run")
        has_colour = bool((self.saved_setup or {}).get("sampled_hsv"))
        put(frame, "Saved setup found", (x0 + 18, y0 + 32), WHITE, 0.7, 2)
        put(frame, f"{'Corners and arrow colour' if has_colour else 'Corners'} from {saved_at}",
            (x0 + 18, y0 + 58), GREY, 0.48)
        put(frame, "[Enter] use it", (x0 + 18, y0 + 90), OK_GREEN, 0.58)
        put(frame, "[R] set up again", (x0 + 18, y0 + 114), YELLOW, 0.58)

    def _draw_help_card(self, frame) -> None:
        height, width = frame.shape[:2]
        card_w, card_h = 400, 64 + 26 * len(HELP_LINES)
        x0, y0 = (width - card_w) // 2, max(4, (height - card_h) // 2)
        shade(frame, 0, 0, width, height, 0.45)
        cv2.rectangle(frame, (x0, y0), (x0 + card_w, y0 + card_h), (29, 24, 22), -1)
        cv2.rectangle(frame, (x0, y0), (x0 + card_w, y0 + card_h), (71, 63, 58), 1)
        put(frame, "Keys", (x0 + 22, y0 + 36), WHITE, 0.75, 2)
        for i, (key, text) in enumerate(HELP_LINES):
            y = y0 + 66 + i * 26
            cv2.rectangle(frame, (x0 + 22, y - 15), (x0 + 122, y + 5), (55, 47, 42), -1)
            put(frame, key, (x0 + 30, y), YELLOW, 0.47)
            put(frame, text, (x0 + 136, y), GREY, 0.5)

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

    def _sync_trackbars(self) -> None:
        """Move the sliders to the current tolerances (after loading a saved setup)."""
        try:
            cv2.setTrackbarPos("H tol", MASK_WINDOW_NAME, self.h_tol)
            cv2.setTrackbarPos("S tol", MASK_WINDOW_NAME, self.s_tol)
            cv2.setTrackbarPos("V tol", MASK_WINDOW_NAME, self.v_tol)
        except cv2.error:
            pass    # window not created yet (e.g. during tests)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def process_frame(self, frame, now: float):
        """Detect the car, work out the motor parameters and draw the overlay.

        Returns (parameters to send or None while setting up, detection mask,
        the picture to show with its status bars).
        """
        self._last_frame = frame
        car_px = None
        distance_mm = None
        valid = False
        mask = None
        params = None

        if self.state == "TRACKING":
            car_px, mask = self.locate_car(frame, now)
            if car_px is not None:
                distance_mm = self.compute_distance(self.track_mm)
                valid = True
                self.last_car_px = car_px
                self.last_seen_time = now
                self.last_distance_mm = distance_mm
            params = self.motor_params_for(distance_mm, valid, now)
        elif self.sampled_hsv is not None:
            # Preview the mask live while the user adjusts tolerances.
            _, mask = self.detect_arrow(frame)

        self.update_run(distance_mm, valid, now)
        canvas = self.draw_overlay(frame, car_px, distance_mm, valid, mask, params or (0, 0, 0, 0))
        return params, mask, canvas

    def handle_key(self, key: int) -> bool:
        """Act on a key press. Returns False when the tracker should quit."""
        now = time.monotonic()
        if key == 255:
            return True                          # no key pressed

        # Typing a team name: every key goes to the name
        if self.name_buffer is not None:
            if key in KEY_ENTER:
                self.commit_name()
            elif key == KEY_ESCAPE:
                self.name_buffer = None
            elif key in KEY_BACKSPACE:
                self.name_buffer = self.name_buffer[:-1]
            elif 32 <= key <= 126 and len(self.name_buffer) < lb.MAX_TEAM_NAME:
                self.name_buffer += chr(key)
            return True

        if key == ord("q"):
            return False
        if key == ord("n"):
            self.begin_name_entry()
            return True
        if key == KEY_SPACE:
            self.start_run(now)
            return True
        if key == ord("x"):
            self.abort_run(now)
            return True
        if key == ord("u"):
            self.undo_last_result(now)
            return True
        if key == ord("s"):
            if self.sound is None or self.sound.command is None:
                self.say("Sound is not available on this computer.", now)
            else:
                self.say("Sound on." if self.sound.toggle() else "Sound off.", now, 2.0)
            return True
        if self.run_phase in ("running", "countdown") and key in (ord("r"), ord("c"), ord("d"), ord("g"),
                                                   *KEY_BACKSPACE):
            self.say("Not during a run. Press X to abort the run first.", now)
            return True
        if key == ord("h"):
            self.show_help = not self.show_help
        elif key in KEY_ENTER:
            if self.state == "RESUME":
                self.accept_saved_setup()
            elif self.state == "GRID_CHECK":
                self.confirm_grid()
        elif key in KEY_BACKSPACE:
            self.undo_corner()
        elif key == ord("r"):
            self.restart_setup()
        elif key == ord("z"):
            self.show_zones = not self.show_zones
        elif key == ord("m"):
            self.show_mask = not self.show_mask
        elif key == ord("g"):
            if self.homography is not None and self.state in ("TRACKING", "SET_DESTINATION"):
                self.use_fixed_destination()
                self.state = "TRACKING"
        elif key == ord("d"):
            if self.homography is not None and self.state == "TRACKING":
                self.destination_mm = None
                self.destination_px = None
                self.zone_rings = []
                self.state = "SET_DESTINATION"
                print("Click to set new destination")
        elif key == ord("c"):
            if self.homography is not None and self.state in ("TRACKING", "SET_DESTINATION"):
                self.state = "PICK_COLOR"
                print("Click the car to sample its colour.")
        return True

    def start_leaderboard(self, port: int, sound: bool = True) -> None:
        board, live = lb.Leaderboard(), lb.LiveState()
        self.sound = sounds.SoundPlayer(enabled=sound)
        if lb.start_server(board, live, port) is None:
            print("Runs will still be timed and saved, without the leaderboard page.")
        else:
            print(f"Leaderboard page:  http://localhost:{port}/         (put this on the portrait monitor)")
            print(f"Manage page:       http://localhost:{port}/manage   (delete, restore, rename teams)")
        self.attach_leaderboard(board, live)

    def run(self):
        self.connect_serial()
        self.open_camera()

        # The tolerance sliders need a window of their own; the mask also shows
        # as an inset in the main view. Create this window FIRST so that the
        # main window, created after, comes to the front on macOS.
        cv2.namedWindow(MASK_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("H tol", MASK_WINDOW_NAME, self.h_tol, MAX_H_TOL,
                           self._on_h_tol)
        cv2.createTrackbar("S tol", MASK_WINDOW_NAME, self.s_tol, MAX_SV_TOL,
                           self._on_s_tol)
        cv2.createTrackbar("V tol", MASK_WINDOW_NAME, self.v_tol, MAX_SV_TOL,
                           self._on_v_tol)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self._on_window_mouse)

        cv2.moveWindow(WINDOW_NAME, 50, 50)
        cv2.moveWindow(MASK_WINDOW_NAME, 800, 50)

        # Prime both windows so cocoa actually renders them before the first
        # cap.read() (works around AUTOSIZE+trackbar quirks).
        placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.imshow(WINDOW_NAME, placeholder)
        cv2.imshow(MASK_WINDOW_NAME, placeholder[:, :, 0])
        cv2.waitKey(1)

        last_send = 0.0
        send_interval = 0.05  # 20 Hz
        first_frame = True

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                frame = self.fit_frame(frame)
                if first_frame:
                    first_frame = False
                    self.offer_saved_setup(frame)

                now = time.monotonic()
                params, mask, canvas = self.process_frame(frame, now)

                if params is not None and now - last_send >= send_interval:
                    last_send = now
                    self.send_motor_params(*params)

                if mask is None:
                    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                cv2.imshow(MASK_WINDOW_NAME, mask)
                cv2.imshow(WINDOW_NAME, canvas)

                if not self.handle_key(cv2.waitKey(1) & 0xFF):
                    break
        except KeyboardInterrupt:
            print("\nInterrupted — shutting down.")
        finally:
            if self.state == "TRACKING":
                self.save_setup()        # keeps any slider changes made during the run
            self.send_motor_params(0, 0, 0, 0)
            time.sleep(0.1)
            if self.ser:
                self.ser.close()
            if self.cap:
                self.cap.release()
            self.play(None)
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
    parser.add_argument("--leaderboard-port", type=int, default=lb.DEFAULT_PORT,
                        help="Port for the leaderboard web page")
    parser.add_argument("--no-sound", action="store_true",
                        help="Start with the run sounds switched off ([S] toggles them)")
    parser.add_argument("--no-leaderboard", action="store_true",
                        help="Track only: no teams, timer or leaderboard page")
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
    if not args.no_leaderboard:
        tracker.start_leaderboard(args.leaderboard_port, sound=not args.no_sound)
    tracker.run()


if __name__ == "__main__":
    main()
