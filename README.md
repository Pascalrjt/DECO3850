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

Notes:
- The sketch uses `AccelStepper::HALF4WIRE` with pin order `IN1, IN3, IN2, IN4`.
- The left motor direction is inverted in software so both sides move forward together.

## Controller ESP32 Pinout

Board role:
- Handheld transmitter
- MAC: `14:33:5C:25:5B:48`
- Sketch: `Steering/Pot_Controller_ESPNow/Pot_Controller_ESPNow.ino`

### Potentiometers
- `GPIO 34` -> `STEERING_POT` wiper
- `GPIO 35` -> `THROTTLE_POT` wiper
- `GPIO 32` -> `BRAKE_REVERSE_POT` wiper

### Pot Wiring
- Pot outer pin -> `3.3V`
- Other outer pin -> `GND`
- Pot center pin / wiper -> assigned GPIO above

Notes:
- `GPIO 34` and `GPIO 35` are input-only ADC pins, which is appropriate for the steering and throttle potentiometers.
- The controller sketch calibrates steering center at startup, so leave the steering pot centered while powering on.

## Wireless Link

- Protocol: `ESP-NOW`
- Channel: `6`
- The controller sends steering, throttle, brake, and reverse-arm state to the car.
- The car accepts packets from the controller MAC above and stops if packets time out.

## Legacy Joystick Test Pinout

These pins are still used by the older joystick-based test sketch and are not used by the wireless RC controller pair:

- `GPIO 35` -> `JOY_X` (`VRX`)
- `GPIO 34` -> `JOY_Y` (`VRY`)

# Vibration

## Vibration Motor Test Pinout

Board role:
- Vibration motor controller
- Sketch: `Vibration/spinnything.ino`

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
