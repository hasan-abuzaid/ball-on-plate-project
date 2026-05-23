# Ball & Plate Balancing System

A closed-loop visual servoing system that balances a ping pong ball on a 30×30 cm plate using computer vision, LQR optimal control, and exact inverse kinematics. Built as a combined project for **Robotics** and **Control Systems II** at German Jordanian University (GJU).

![Status](https://img.shields.io/badge/status-completed-brightgreen)
![Controller](https://img.shields.io/badge/controller-LQR-blue)
![Platform](https://img.shields.io/badge/hardware-Arduino%20Mega-orange)
![Vision](https://img.shields.io/badge/vision-OpenCV-green)

---

https://github.com/user-attachments/assets/25094b64-cf86-401e-aa2f-fe0da3c0cd70

## Overview

A camera mounted above the plate tracks the ball in real-time using HSV color detection. A Python controller running on the host PC estimates ball position and velocity, computes the required plate tilt via LQR state feedback, maps that tilt to servo angles through exact inverse kinematics, and sends commands to an Arduino over serial.

The system evolved through multiple iterations — from a basic linear mapping that vibrated violently, to a robust LQR controller with low-pass velocity filtering and exact IK that handles the nonlinear servo-to-plate geometry.

## System Architecture

```
    USB Webcam (HSV Ball Detection)
              │
              ▼
     ┌─────────────────────┐
     │   Python Controller  │
     │  ┌────────────────┐  │
     │  │ OpenCV Vision   │  │
     │  │ Velocity Est.   │  │
     │  │ LQR Control Law │  │
     │  │ Inverse Kinem.  │  │
     │  │ Low-Pass Filter │  │
     │  └────────────────┘  │
     └──────────┬───────────┘
                │ Serial (115200 baud)
                │ Protocol: "SX<angle> SY<angle>\n"
                ▼
         Arduino Mega 2560
     ┌─────────────────────┐
     │ Servo X (Pin 9)     │
     │ Servo Y (Pin 10)    │
     │ Safety clamps       │
     │ Calibrated offsets   │
     └──────────┬──────────┘
                │
                ▼
         30×30 cm Plate
      (2-DOF tilt via linkages)
```

## The "Golden" Configuration

| Parameter | Value |
|---|---|
| LQR Gains (K) | `[-70.71, -64.54, 205.87, 20.32]` |
| Plate Radius | 13.5 cm |
| Servo Horn Length | 5.5 cm |
| Hardware Offsets | X: 105° / Y: 100° |
| Low-Pass Smooth Factor | 0.6 (60% old, 40% new) |
| Servo Safety Limits | X: 50°–150° / Y: 60°–150° |

## Repository Structure

```
ball-on-plate/
├── README.md
├── LICENSE
├── .gitignore
├── python/
│   ├── robot_main.py           # Full controller: vision + LQR + IK + serial
│   ├── ik_calibrator.py        # Standalone IK verification tool (no camera)
│   └── README.md
├── firmware/
│   ├── servo_controller.ino    # Arduino servo driver with safety clamps
│   └── README.md
├── matlab/
│   ├── lqr_analysis.m          # State-space model, LQR design, stability proof
│   ├── lqr_project.m           # Full project analysis with real parameters
│   └── README.md
├── docs/
│   ├── control_theory.md       # Plant model, LQR derivation, stability analysis
│   ├── kinematics.md           # Forward & inverse kinematics derivation
│   ├── engineering_log.md      # Problems encountered and how they were solved
│   └── hardware_bom.md         # Bill of materials & wiring
└── media/                      # Photos, simulation plots, demo videos
```

## Engineering Challenges Solved

Building this system was not a smooth ride. Key problems and their solutions:

**The Vibration Problem** — Raw LQR outputs caused servos to slam back and forth violently. Solved by implementing a low-pass filter (`new = old × 0.6 + target × 0.4`) that acts as an electronic shock absorber, smoothing the control signal without killing responsiveness.

**The Y-Axis Runaway** — The math was correct but the physical motor was oriented in reverse. The system pushed the ball off instead of catching it. Fixed by flipping the Y-axis scaling by −1.0.

**The Edge "Lazy" Phenomenon** — A linear servo-to-plate angle approximation failed at steep tilt angles because the mechanical ratio between plate arm (13.5 cm) and servo horn (5.5 cm) is large. Solved by implementing exact inverse kinematics using `arcsin`.

**Port Locking & Circular Imports** — Spyder IDE locked the COM port, and a file accidentally named `serial.py` caused Python's `import serial` to import itself. Fixed with kernel restarts, proper file naming, and a robust `try/finally` shutdown block.

## Quick Start

### 1. Flash the Arduino
Upload `firmware/servo_controller.ino` to Arduino Mega via Arduino IDE. The servos will move to their calibrated neutral positions (X: 105°, Y: 100°).

### 2. Calibrate Linkage Geometry
Run `python/ik_calibrator.py` to verify the inverse kinematics against your physical build. Input plate angles and confirm the servos move correctly before running the full controller.

### 3. Run the Controller
```bash
pip install opencv-python imutils pyserial numpy
python python/robot_main.py
```
Place the ball on the plate. The system will track and balance it at the center. Press `q` to quit cleanly.

### 4. Validation Tests

| Test | Procedure | Pass Criteria |
|---|---|---|
| Steady-State Drift | Place ball at center, hands off | Motors silent, ball stays put |
| Impulse Response | Flick the ball | Returns to center within ≤2 oscillations |
| Step Response | Drop ball from 15 cm edge | Settles to center in <3 seconds |
| Blind Spot | Cover camera | System freezes safely (no erratic motion) |

### 5. Run MATLAB Analysis
Open `matlab/lqr_analysis.m` or `matlab/lqr_project.m` in MATLAB to verify controller design and view simulation results.

## Control Design Summary

The system is modeled as **two independent ball-and-beam systems** (X and Y axes are decoupled under small-angle assumptions). For each axis:

**State vector:** `x = [position, velocity, plate_angle, angular_velocity]`

**Plant dynamics (linearized):** `ẍ = H · θ` where `H = −mg / (J/R² + m)`

**Controller:** LQR full-state feedback `u = −Kx + N̄r` where `N̄` is the reference scaling gain for zero steady-state error.

**Stability:** The open-loop system has poles at the origin (marginally unstable). The LQR controller places all closed-loop poles in the left half-plane, guaranteeing asymptotic stability. Controllability rank = 4 (full rank) confirms the system is fully controllable.

See [`docs/control_theory.md`](docs/control_theory.md) for the full derivation.

## Hardware

| Component | Specification |
|---|---|
| Plate | 30×30 cm flat surface |
| Ball | Ping pong ball (40 mm, ~2 g) |
| Servos | 2× hobby servos (SG90 / MG996R) |
| Servo horn length | 5.5 cm |
| Linkage rod length | 11 cm (servo to plate) |
| Controller | Arduino Mega 2560 |
| Camera | USB webcam (640×480+) |
| Frame | Rigid mounting structure |

## Courses

- **Robotics** — Forward/inverse kinematics, servo actuation, system integration
- **Control Systems II** — State-space modeling, LQR design, controllability, stability analysis

## Author

**Hasan Abuzaid**
Mechatronics Engineering — German Jordanian University (GJU)

## License

[MIT License](LICENSE)
