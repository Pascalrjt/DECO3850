# MazeRoomba

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
- The car reads both axes every 40 ms and sends a `FeedbackPacket` back to the controller indicating whether the joystick is deflected past its deadzone.

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

### Haptic Motor Drivers

Three vibration motors total — two for wall feedback, one for proximity. A single DFRobot dual driver only exposes two channels (M1/M2), so this build needs **two dual drivers** (one channel left spare) or a quad driver.

#### Wall feedback — two motors (driven together)
- Motor A: `GPIO 23` -> `PWM`, `GPIO 19` -> `DIR`
- Motor B: `GPIO 13` -> `PWM`, `GPIO 27` -> `DIR`
- Each motor's wires -> a driver channel's `+` / `-` terminals

#### Proximity feedback — one motor
- `GPIO 25` -> `PWM`
- `GPIO 26` -> `DIR`
- Motor wires -> a driver channel's `+` / `-` terminals

#### Driver power (wire for each driver)
- Driver `GND` -> ESP32 `GND` (common ground)
- Driver `VCC` -> `5V` (logic supply, can share USB 5V)
- Driver `VM` -> external motor power supply positive (6-12V typical)
- Motor supply `GND` -> driver `GND` (common ground)

Notes:
- `GPIO 34` and `GPIO 35` are input-only ADC pins, which is appropriate for the steering and throttle potentiometers.
- The controller sketch calibrates steering center at startup, so leave the steering pot centered while powering on.
- The two wall haptic motors run together while the car-side joystick is deflected and stop immediately when the joystick returns to center or the feedback link times out.
- The proximity haptic motor pulses at a rate and strength dictated by `ProximityPacket` messages from the bridge ESP32. Faster pulses mean the car is closer to the destination.
- The haptic GPIOs (13/19/23/25/26/27) are all general-purpose pins with no ESP32 boot-strapping role, so a driver input cannot block start-up. Avoid `GPIO 12` for these — it selects flash voltage at boot, and a driver holding it high can stop the board from booting.

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
- The car sends `FeedbackPacket` (joystick deflection state, X/Y values) back to the controller every 40 ms.
- The bridge sends `ProximityPacket` (motor strength, pulse timing, flags) to the controller at ~20 Hz.
- All three packet types have distinct sizes (11 / 9 / 10 bytes) so the controller dispatches by `dataLen`.
- The car stops driving if controller packets time out (250 ms).
- The controller stops each haptic motor independently if the corresponding packet stream times out (500 ms).

## Proximity Tracking (PC-side)

- Script: `Vision/proximity_tracker.py`
- Dependencies: `Vision/requirements.txt` (`pip install -r Vision/requirements.txt`)
- An overhead webcam views the 1.2 m × 1.2 m maze. The overseer calibrates four maze corners (click TL, TR, BR, BL) and clicks to set the destination.
- A coloured arrow on the car roof is detected via HSV thresholding. The centroid is mapped through a homography to maze-millimetre coordinates, and the Euclidean distance to the destination is converted to motor parameters (strength and pulse rate).
- Motor parameters are sent as framed binary packets over USB serial to the bridge ESP32, which rebroadcasts them via ESP-NOW.

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
- Vibration motor controller (ESP-NOW receiver)
- MAC: `c0:cd:d6:81:ff:80`
- Sketch: `Vibration/spinnything/spinnything.ino`

### Motor Driver 1 (all channels ESP-NOW triggered)
- `GPIO 25` -> `M1_PWM`
- `GPIO 26` -> `M1_DIR`
- `GPIO 17` -> `M2_PWM`
- `GPIO 27` -> `M2_DIR`

### Motor Driver 2 (all channels ESP-NOW triggered)
- `GPIO 5`  -> `M3_PWM`
- `GPIO 13` -> `M3_DIR`
- `GPIO 23` -> `M4_PWM`
- `GPIO 19` -> `M4_DIR`

Notes:
- All four motors are triggered wirelessly by ButtonESP over ESP-NOW.
- Each motor runs while its button is held and stops when the button is released — START on the press edge, STOP on the release edge.
- Motor output is a fixed `analogWrite(pwm, 150)` with `DIR` held HIGH; there is no on-board timeout, so the motor only stops when a STOP command arrives.

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
- The falling edge (press) sends a START byte (`0x01`–`0x04`) and the rising edge (release) sends the matching STOP byte (`0x11`–`0x14`), so the motor is active for exactly as long as the button is held.
- The loop polls every 10 ms with edge detection against the previous reading — no debouncing beyond that delay.

## Vibration Wireless Link

- Protocol: `ESP-NOW`
- ButtonESP sends a `TriggerPacket` (1 byte) to MotorESP on each button edge — one packet on press, one on release.
- START command byte mapping: `0x01` = Motor 1, `0x02` = Motor 2, `0x03` = Motor 3, `0x04` = Motor 4.
- STOP command byte mapping: `0x11` = Motor 1, `0x12` = Motor 2, `0x13` = Motor 3, `0x14` = Motor 4 (high nibble `0x10` is OR-ed onto the motor id).
- MotorESP runs the matching motor on START and stops it on STOP; there is no automatic timeout, so a lost STOP packet will leave the motor running until the next STOP arrives.

