# AGENTS.md

## Project Scope

This repository contains Arduino and ESP32 sketches intended to be compiled and uploaded through the Arduino IDE.

## Sketch Layout Rules

- Each Arduino sketch must live in a directory with the same name as the main `.ino` file.
- When creating a new sketch, use the Arduino IDE-compatible structure from the start.
- When editing an existing sketch, preserve that directory/name pairing.

Examples:

- `MySketch/MySketch.ino`
- `RobotDrive/RobotDrive.ino`

## Multi-Device And Wiring Rules

- If a sketch or set of sketches involves multiple devices, verify the IO assignments against the repository documentation before changing code.
- If wiring, pin usage, or device responsibilities change, update the IO mappings in `README.md` in the same task.
- Treat `README.md` as the project-level source of truth for shared pin assignments.

## Current IO Mapping Reference

The current shared mappings are documented in `README.md` and presently cover:

- Left stepper motor pins
- Right stepper motor pins
- Joystick pins

## Editing Expectations

- Prefer changes that remain directly compatible with Arduino IDE workflows.
- Do not move sketches into layouts that break Arduino IDE sketch discovery.
- Keep multi-device code and documentation aligned so the codebase and wiring notes do not drift.
- When uploading with `arduino-cli`, always use `115200` baud.
