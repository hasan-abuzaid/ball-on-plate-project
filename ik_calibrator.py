# -*- coding: utf-8 -*-
"""
Inverse Kinematics Calibrator
==============================
Standalone tool to verify the servo-to-plate linkage geometry
without needing the camera. Input a desired plate angle and
confirm the servos move to the correct physical position.

Author:  Hasan Abuzaid
Course:  Robotics & Control Systems II — GJU

Usage:
    python ik_calibrator.py
    Enter plate angle in degrees, or 'q' to quit.
"""

import serial
import time
import math

# --- Configuration ---
PORT = "COM3"
OFFSET_X = 105      # Servo X neutral [deg]
OFFSET_Y = 100      # Servo Y neutral [deg]
R_PLATE = 13.5      # Plate arm radius [cm]
R_SERVO = 5.5       # Servo horn length [cm]


def get_ik_angle(angle_degrees, offset):
    """
    Compute servo angle from desired plate tilt angle.

    Args:
        angle_degrees: Desired plate tilt [deg]
        offset: Servo neutral position [deg]

    Returns:
        Servo command angle [int, deg]
    """
    h = R_PLATE * math.sin(math.radians(angle_degrees))
    servo_deg = math.degrees(math.asin(max(-1.0, min(1.0, h / R_SERVO))))
    return int(offset - servo_deg)


# --- Main ---
ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(2)

try:
    print("IK Calibrator — Enter plate angle in degrees ('q' to quit)")
    print(f"Geometry: R_plate={R_PLATE} cm, R_servo={R_SERVO} cm")
    print(f"Offsets:  X={OFFSET_X}°, Y={OFFSET_Y}°\n")

    while True:
        val = input("Angle (deg): ")
        if val.strip().lower() == 'q':
            break

        try:
            angle = float(val)
        except ValueError:
            print("  Invalid input. Enter a number or 'q'.")
            continue

        sx = get_ik_angle(angle, OFFSET_X)
        sy = get_ik_angle(angle, OFFSET_Y)

        print(f"  → Servo X: {sx}°  |  Servo Y: {sy}°")
        ser.write(f"SX{sx} SY{sy}\n".encode())

finally:
    ser.close()
    print("Serial closed.")
