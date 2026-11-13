# ESP32 Board Log

Each board was tested on 2026-11-13 by uploading a self-test sketch through `arduino-cli`. The self-test prints the chip details and the Wi-Fi station MAC, runs a Wi-Fi scan and starts ESP-NOW. All five boards passed.

The MAC constants in every sketch, the README and the wiring diagrams were updated on the same day to match this table.

| Board | Role | MAC | Sketch | Chip | Flash | Self-test |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Controller | `14:33:5C:61:11:40` | `Steering/Pot_Controller_Haptic_ESPNow` | ESP32-D0WD-V3 rev 3.1 | 4 MB | PASS (11 Wi-Fi networks, ESP-NOW OK) |
| 2 | MotorESP | `68:FE:71:2C:78:BC` | `Vibration/spinnything` | ESP32-D0WD-V3 rev 3.1 | 4 MB | PASS (15 Wi-Fi networks, ESP-NOW OK) |
| 3 | RC car | `68:FE:71:2C:0B:C8` | `MazeRoomba/RC_Car_ESPNow` | ESP32-D0WD-V3 rev 3.1 | 4 MB | PASS (11 Wi-Fi networks, ESP-NOW OK) |
| 4 | Bridge | `68:FE:71:2B:75:D8` | `Bridge/ProximityBridge_ESPNow` | ESP32-D0WD-V3 rev 3.1 | 4 MB | PASS (12 Wi-Fi networks, ESP-NOW OK) |
| 5 | ButtonESP | `C0:CD:D6:81:FF:80` | `Vibration/buttonTrigger` | ESP32-D0WD-V3 rev 3.1 | 4 MB | PASS (14 Wi-Fi networks, ESP-NOW OK) |

Label each board with its role, since the code finds the boards by MAC address. If a board is swapped, read its MAC with `MacAddress/GetMacAddress` and update the constants in the sketches that send to it or receive from it.
