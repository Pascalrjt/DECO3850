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

### Haptic Motor Driver (DFRobot dual driver, Motor 1 channel)
- `GPIO 25` -> `M1_PWM`
- `GPIO 26` -> `M1_DIR`
- Driver `GND` -> ESP32 `GND` (common ground)
- Driver `VCC` -> `5V` (logic supply, can share USB 5V)
- Driver `VM` -> external motor power supply positive (6-12V typical)
- Motor supply `GND` -> driver `GND` (common ground)
- Motor wires -> driver `M1+` / `M1-` terminals

Notes:
- `GPIO 34` and `GPIO 35` are input-only ADC pins, which is appropriate for the steering and throttle potentiometers.
- The controller sketch calibrates steering center at startup, so leave the steering pot centered while powering on.
- The haptic motor runs at PWM duty 150/255 while the car-side joystick is deflected, and stops immediately when the joystick returns to center or the feedback link times out.

## Wireless Link

- Protocol: `ESP-NOW`
- Channel: `6`
- The controller sends `ControlPacket` (steering, throttle, brake, reverse-arm) to the car every 40 ms.
- The car sends `FeedbackPacket` (joystick deflection state, X/Y values) back to the controller every 40 ms.
- Both sides distinguish packet types by length and reject unexpected sizes.
- The car stops driving if controller packets time out (250 ms).
- The controller stops the haptic motor if feedback packets time out (500 ms).

## Legacy Joystick Test Pinout

The older test sketch at `MazeRoomba/Stepper-test/Joystick/Joystick.ino` used `GPIO 39` / `GPIO 36` for the joystick. The RC car now uses `GPIO 35` / `GPIO 34` for the same joystick module (see the Joystick subsection above).

# Vibration

## Vibration Motor Test Pinout

Board role:
- Vibration motor controller
- Sketch: `Vibration/spinnything/spinnything.ino`

### Motor Driver 1
- `GPIO 25` -> `M1_PWM`
- `GPIO 26` -> `M1_DIR`
- `GPIO 14` -> `BTN_1`
- `GPIO 17` -> `M2_PWM`
- `GPIO 27` -> `M2_DIR`
- `GPIO 12` -> `BTN_2`

### Motor Driver 2
- `GPIO 5` -> `M3_PWM`
- `GPIO 13` -> `M3_DIR`
- `GPIO 33` -> `BTN_3`
- `GPIO 23` -> `M4_PWM`
- `GPIO 19` -> `M4_DIR`
- `GPIO 32` -> `BTN_4`

Notes:
- The sketch uses pushbuttons with `INPUT_PULLUP` to independently trigger each motor.
- When triggered, each motor runs for a fixed duration of 1000ms.
