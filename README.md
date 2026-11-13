# MazeRoomba

## Wiring Diagrams

Colour-coded wiring diagrams for all five boards are in `wiring-diagrams/`. Open them in a browser.

- `wiring-diagrams/index-motoresp-wall-haptics.html` matches the current code: wall haptics on the MotorESP, one driver on the controller.
- `wiring-diagrams/index.html` shows the earlier design, with the two wall motors on the controller.

## RC Car ESP32 Pinout

Board role:
- Receiver / vehicle controller
- MAC: `14:33:5C:61:11:40`
- Sketch: `MazeRoomba/RC_Car_ESPNow/RC_Car_ESPNow.ino`

### Left Stepper Motor (Motor 1)
- `GPIO 27` -> `M1_IN1`
- `GPIO 14` -> `M1_IN2`
- `GPIO 12` -> `M1_IN3`
- `GPIO 13` -> `M1_IN4`

### Right Stepper Motor (Motor 2)
- `GPIO 26` -> `M2_IN1`
- `GPIO 25` -> `M2_IN2`
- `GPIO 33` -> `M2_IN3`
- `GPIO 32` -> `M2_IN4`

### Joystick
- `GPIO 35` -> `JOY_VRX` (X-axis wiper)
- `GPIO 34` -> `JOY_VRY` (Y-axis wiper)
- `SW` -> not connected
- `+5V` / `VCC` -> `3.3V`
- `GND` -> `GND`

Notes:
- The sketch uses `AccelStepper::HALF4WIRE` with pin order `IN1, IN3, IN2, IN4`.
- The left motor direction is inverted in software so both sides move forward together.
- The joystick center is calibrated at startup. Leave the joystick centered while powering on.
- The joystick is the car's wall-bump sensor. The car reads both axes every 40 ms and sends a `FeedbackPacket` to the **MotorESP** (`C0:CD:D6:81:FF:80`) indicating whether the joystick is deflected past its deadzone. The MotorESP pulses all four of its vibration motors while it is deflected (see [Vibration](#vibration)).

## Controller ESP32 Pinout

Board role:
- Handheld transmitter
- MAC: `68:FE:71:2B:75:D8`
- Sketch: `Steering/Pot_Controller_Haptic_ESPNow/Pot_Controller_Haptic_ESPNow.ino` (with haptic feedback)
- Legacy sketch (no haptic): `Steering/Pot_Controller_ESPNow/Pot_Controller_ESPNow.ino`

### Potentiometers
- `GPIO 34` -> `STEERING_POT` wiper
- `GPIO 35` -> `THROTTLE_POT` wiper
- `GPIO 32` -> `BRAKE_REVERSE_POT` wiper

### Pot Wiring
- Pot outer pin -> `3.3V`
- Other outer pin -> `GND`
- Pot center pin / wiper -> assigned GPIO above

### Haptic Motor Driver

One vibration motor for proximity feedback, on one DFRobot dual driver (`M1` channel used, `M2` spare). Wall feedback has moved to the MotorESP, so the controller no longer has wall motors.

#### Proximity feedback — one motor
- `GPIO 25` -> `PWM` (driver `PWM1`)
- `GPIO 26` -> `DIR` (driver `DIR1`)
- Motor wires -> driver `M1+` / `M1-`

#### Driver power
- Driver `GND` -> ESP32 `GND` (common ground)
- Driver `VCC` -> `3.3V` (logic supply; at 5V the driver needs 3.5V for a logic high and the ESP32 outputs 3.3V)
- Driver `VM` -> external motor power supply positive (6-12V typical)
- Motor supply `GND` -> driver `GND` (common ground)

Notes:
- `GPIO 34` and `GPIO 35` are input-only ADC pins, which is appropriate for the steering and throttle potentiometers.
- The controller sketch calibrates steering center at startup, so leave the steering pot centered while powering on.
- The proximity haptic motor pulses according to `ProximityPacket` messages from the bridge ESP32. Distance is quantised into 200 mm buckets and each bucket fires a fixed 100 ms pulse at full PWM (255); only the gap between pulses changes. Faster pulses mean the car is closer to the destination — see the table under [Proximity Tracking (PC-side)](#proximity-tracking-pc-side).
- The haptic GPIOs (25/26) are general-purpose pins with no ESP32 boot-strapping role, so a driver input cannot block start-up. Avoid `GPIO 12` for haptics — it selects flash voltage at boot, and a driver holding it high can stop the board from booting.

## Bridge ESP32 Pinout

Board role:
- USB-serial to ESP-NOW bridge for proximity data
- MAC: `68:FE:71:2C:0B:C8`
- Sketch: `Bridge/ProximityBridge_ESPNow/ProximityBridge_ESPNow.ino`

No additional GPIO wiring — the bridge only uses USB serial for data input and ESP-NOW for output. Connect it to the PC running `Vision/proximity_tracker.py` via USB.

This MAC is set as `BRIDGE_MAC` in the controller sketch (`Pot_Controller_Haptic_ESPNow.ino`) so the controller can register the bridge as an ESP-NOW peer.

## Wireless Link

- Protocol: `ESP-NOW`
- Channel: `6`
- The controller sends `ControlPacket` (steering, throttle, brake, reverse-arm) to the car every 40 ms.
- The car sends `FeedbackPacket` (joystick deflection state, X/Y values) to the MotorESP every 40 ms.
- The bridge sends `ProximityPacket` (motor strength, pulse timing, flags) to the controller at ~20 Hz.
- ButtonESP sends 1-byte `TriggerPacket`s to the MotorESP on the same channel.
- Packet types have distinct sizes (Control 11, Feedback 9, Proximity 10, Trigger 1 bytes), so receivers dispatch by `dataLen`.
- The car stops driving at once, with no ramp-down, if controller packets time out (250 ms).
- The controller stops the proximity motor if `ProximityPacket`s time out (500 ms).
- The MotorESP stops the wall pulse if `FeedbackPacket`s time out (500 ms).
- All five boards that use ESP-NOW (controller, car, bridge, MotorESP, ButtonESP) must be on channel `6`.

## Proximity Tracking (PC-side)

- Script: `Vision/proximity_tracker.py`
- Dependencies: `Vision/requirements.txt` (`pip install -r Vision/requirements.txt`)
- An overhead webcam views the 1.2 m × 1.2 m maze. The overseer calibrates four maze corners (click TL, TR, BR, BL) and clicks to set the destination.
- A coloured arrow on the car roof is detected via HSV thresholding. The centroid is mapped through a homography to maze-millimetre coordinates, and the Euclidean distance to the destination is bucketed into 200 mm steps to drive the haptic motor.
- Motor parameters are sent as framed binary packets over USB serial to the bridge ESP32, which rebroadcasts them via ESP-NOW.

### Distance → haptic mapping

All pulses fire at PWM `255` with a `100 ms` pulse width; only `pulse_off` (the gap between pulses) changes between buckets. Below the arrival threshold the motor is held solid-on. Tuneables live in `Vision/proximity_tracker.py` (`PROX_STRENGTH`, `PROX_PULSE_ON_MS`, `PROX_DISTANCE_BUCKETS`).

| Distance (mm) | `pulse_on` | `pulse_off` | Rate |
| --- | --- | --- | --- |
| < 50 (arrived) | 500 ms | 0 ms | solid on |
| 50–200 | 100 ms | 100 ms | ~5.0 Hz |
| 200–400 | 100 ms | 220 ms | ~3.1 Hz |
| 400–600 | 100 ms | 400 ms | ~2.0 Hz |
| 600–800 | 100 ms | 700 ms | ~1.25 Hz |
| 800–1000 | 100 ms | 1150 ms | ~0.8 Hz |
| 1000–1200 | 100 ms | 1900 ms | ~0.5 Hz |

### Running the tracker

Default invocation (uses the macOS default port `/dev/cu.usbserial-10`):

```
python3 Vision/proximity_tracker.py
```

If the serial port cannot be opened the script prints `Serial connection failed` and continues in vision-only mode — calibration and tracking still work but no packets reach the bridge, so the proximity haptic motor will stay silent.

### CLI flags

- `--port <path>` — serial port for the bridge ESP32. Default `/dev/cu.usbserial-10` (macOS). Use `ls /dev/cu.usbserial-*` to confirm the device name, or `/dev/ttyUSB0` on Linux.
- `--baud <int>` — serial baud rate. Default `115200` (must match the bridge sketch).
- `--camera <int>` — camera device index passed to OpenCV. Default `0`.
- `--hsv-lower H S V` — HSV lower bound for arrow detection. Passing this (or `--hsv-upper`) skips the interactive colour picker.
- `--hsv-upper H S V` — HSV upper bound for arrow detection.
- `--h-tol <int>` — initial hue tolerance for the interactive picker. Default `10`.
- `--s-tol <int>` — initial saturation tolerance for the interactive picker. Default `60`.
- `--v-tol <int>` — initial value tolerance for the interactive picker. Default `60`.

### Runtime keys

- `R` — reset calibration (re-click the four corners).
- `C` — re-pick the car colour.
- `D` — set a new destination.
- `Q` — quit.

## Legacy Joystick Test Pinout

The older test sketch at `MazeRoomba/Stepper-test/Joystick/Joystick.ino` used `GPIO 39` / `GPIO 36` for the joystick. The RC car now uses `GPIO 35` / `GPIO 34` for the same joystick module (see the Joystick subsection above).

# Vibration

## MotorESP Pinout

Board role:
- Vibration motor controller (ESP-NOW receiver for ButtonESP and the RC car's wall feedback)
- MAC: `c0:cd:d6:81:ff:80`
- Sketch: `Vibration/spinnything/spinnything.ino`

### Motor Driver 1
- `GPIO 25` -> `M1_PWM`
- `GPIO 26` -> `M1_DIR`
- `GPIO 17` -> `M2_PWM`
- `GPIO 27` -> `M2_DIR`

### Motor Driver 2
- `GPIO 5`  -> `M3_PWM`
- `GPIO 13` -> `M3_DIR`
- `GPIO 23` -> `M4_PWM`
- `GPIO 19` -> `M4_DIR`

Notes:
- **Buttons:** each motor runs solid (`PWM 150`) while its ButtonESP button is held and stops when it is released — START on the press edge, STOP on the release edge. ButtonESP repeats START every 100 ms while a button is held, and the MotorESP stops a motor if no START arrives for 400 ms (`BUTTON_TIMEOUT_MS`), so a lost packet cannot leave a motor running.
- **Wall feedback:** while the RC car's joystick reports a deflection, all four motors pulse together (`PWM 150`, 100 ms on / 100 ms off). The pulse stops when the joystick re-centres or the car's `FeedbackPacket`s stop for 500 ms. Tuneables: `WALL_PWM_LEVEL`, `WALL_PULSE_ON_MS`, `WALL_PULSE_OFF_MS`, `WALL_TIMEOUT_MS`.
- A held button takes priority over the wall pulse for its own motor.
- Car `FeedbackPacket`s are only accepted from the car MAC `14:33:5C:61:11:40`.

## ButtonESP Pinout

Board role:
- Button transmitter (ESP-NOW sender)
- MAC: `68:FE:71:2C:78:BC`
- Sketch: `Vibration/buttonTrigger/buttonTrigger.ino`

### Buttons
- `GPIO 27` -> `BTN_1` (triggers Motor 1)
- `GPIO 14` -> `BTN_2` (triggers Motor 2)
- `GPIO 12` -> `BTN_3` (triggers Motor 3)
- `GPIO 13` -> `BTN_4` (triggers Motor 4)

Notes:
- All buttons use `INPUT_PULLUP`; connect one side to the GPIO and the other to `GND`.
- The falling edge (press) sends a START byte (`0x01`–`0x04`) and the rising edge (release) sends the matching STOP byte (`0x11`–`0x14`), so the motor is active while the button is held. While held, the START byte is repeated every 100 ms (`HEARTBEAT_MS`).
- The loop polls every 5 ms. A reading must stay unchanged for 20 ms (`DEBOUNCE_MS`) before it counts as a press or release.

## Vibration Wireless Link

- Protocol: `ESP-NOW`, channel `6` (shared with the MazeRoomba boards so the MotorESP can also receive the car's wall feedback)
- ButtonESP sends a `TriggerPacket` (1 byte) to MotorESP on each button edge (one on press, one on release) and repeats the START packet every 100 ms while the button stays held.
- START command byte mapping: `0x01` = Motor 1, `0x02` = Motor 2, `0x03` = Motor 3, `0x04` = Motor 4.
- STOP command byte mapping: `0x11` = Motor 1, `0x12` = Motor 2, `0x13` = Motor 3, `0x14` = Motor 4 (high nibble `0x10` is OR-ed onto the motor id).
- MotorESP runs the matching motor on START and stops it on STOP, or after 400 ms without a repeated START. A lost START is recovered by the next repeat, and a lost STOP by the timeout.
- MotorESP obeys button packets only from the ButtonESP MAC and `FeedbackPacket`s only from the RC car MAC. The controller obeys `ProximityPacket`s only from the bridge MAC. If a board is replaced, update the MAC constants in the receiving sketch.

