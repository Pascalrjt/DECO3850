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
- MAC: `14:33:5C:25:5B:48`
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

### Haptic Motor Driver (DFRobot dual driver)

#### Wall feedback — M1 channel
- `GPIO 25` -> `M1_PWM`
- `GPIO 26` -> `M1_DIR`
- Motor wires -> driver `M1+` / `M1-` terminals

#### Proximity feedback — M2 channel
- `GPIO 27` -> `M2_PWM`
- `GPIO 33` -> `M2_DIR`
- Motor wires -> driver `M2+` / `M2-` terminals

#### Driver power
- Driver `GND` -> ESP32 `GND` (common ground)
- Driver `VCC` -> `5V` (logic supply, can share USB 5V)
- Driver `VM` -> external motor power supply positive (6-12V typical)
- Motor supply `GND` -> driver `GND` (common ground)

Notes:
- `GPIO 34` and `GPIO 35` are input-only ADC pins, which is appropriate for the steering and throttle potentiometers.
- The controller sketch calibrates steering center at startup, so leave the steering pot centered while powering on.
- The wall haptic motor (M1) runs while the car-side joystick is deflected and stops immediately when the joystick returns to center or the feedback link times out.
- The proximity haptic motor (M2) pulses at a rate and strength dictated by `ProximityPacket` messages from the bridge ESP32. Faster pulses mean the car is closer to the destination.

## Bridge ESP32 Pinout

Board role:
- USB-serial to ESP-NOW bridge for proximity data
- MAC: update `BRIDGE_MAC` in the controller sketch with the value printed at bridge startup
- Sketch: `Bridge/ProximityBridge_ESPNow/ProximityBridge_ESPNow.ino`

No additional GPIO wiring — the bridge only uses USB serial for data input and ESP-NOW for output. Connect it to the PC running `Vision/proximity_tracker.py` via USB.

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
- Each motor runs for 1000ms per trigger and stops automatically.
- If a motor is already running, a second trigger for the same motor is ignored until it stops.

## ButtonESP Pinout

Board role:
- Button transmitter (ESP-NOW sender)
- MAC: `68:fe:71:2b:75:d8`
- Sketch: `Vibration/buttonTrigger/buttonTrigger.ino`

### Buttons
- `GPIO 27` -> `BTN_1` (triggers Motor 1)
- `GPIO 14` -> `BTN_2` (triggers Motor 2)
- `GPIO 12` -> `BTN_3` (triggers Motor 3)
- `GPIO 13` -> `BTN_4` (triggers Motor 4)

Notes:
- All buttons use `INPUT_PULLUP`; connect one side to the GPIO and the other to `GND`.
- A press sends a one-byte command to MotorESP on the falling edge only.

## Vibration Wireless Link

- Protocol: `ESP-NOW`
- ButtonESP sends a `TriggerPacket` (1 byte) to MotorESP on each button press.
- Command byte mapping: `0x01` = Motor 1, `0x02` = Motor 2, `0x03` = Motor 3, `0x04` = Motor 4.
- MotorESP starts the corresponding motor on receipt and stops it automatically after 1000ms.

