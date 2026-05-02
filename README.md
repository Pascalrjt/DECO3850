# MazeRoomba

## IO Pin Mappings

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

### Potentiometer Controller
- `GPIO 34` -> `STEERING_POT`
- `GPIO 35` -> `THROTTLE_POT`
- `GPIO 32` -> `BRAKE_REVERSE_POT`

### Joystick
- `GPIO 35` -> `JOY_X` (`VRX`)
- `GPIO 34` -> `JOY_Y` (`VRY`)

## Wireless Roles

- RC car receiver ESP32 MAC: `14:33:5C:61:11:40`
- Controller transmitter ESP32 MAC: `14:33:5C:25:5B:48`
- `MazeRoomba/RC_Car_ESPNow/RC_Car_ESPNow.ino` uses the stepper pin mapping above and receives control packets over `ESP-NOW`.
- `Steering/Pot_Controller_ESPNow/Pot_Controller_ESPNow.ino` reads the three potentiometers above and sends control packets over `ESP-NOW`.
- The older joystick mapping remains documented for the existing joystick test sketch, but the wireless RC car pair ignores the joystick input.
