# -*- coding: utf-8 -*-
"""
Ball & Plate Controller — Main Loop
====================================
Handles vision (OpenCV), state estimation, LQR control,
inverse kinematics, and serial communication to Arduino.

Author:  Hasan Abuzaid
Course:  Robotics & Control Systems II — GJU

Usage:
    python robot_main.py

Press 'q' to quit cleanly.
"""

import cv2
import imutils
import numpy as np
import time
import serial
import math

# ============================================================
#  CONFIGURATION
# ============================================================

# --- Serial ---
PORT = "COM3"
BAUD = 115200

# --- Servo Calibration ---
OFFSET_X = 105          # Neutral angle for X servo [deg]
OFFSET_Y = 100          # Neutral angle for Y servo [deg]

# --- Mechanical Geometry ---
R_PLATE = 13.5          # Plate arm radius [cm]
R_SERVO = 5.5           # Servo horn length [cm]

# --- Control ---
LQR_SCALING = -1.0      # Sign correction for physical motor orientation
SMOOTH_FACTOR = 0.6     # Low-pass filter: 0.6 = 60% old + 40% new
INVERT_Y_AXIS = False   # Set True if Y-axis motor is physically reversed

# --- Safety Clamps (must match Arduino firmware) ---
X_MIN, X_MAX = 50, 150
Y_MIN, Y_MAX = 60, 150

# --- Vision ---
CAM_IDX = 1             # Camera index (try 0 if 1 doesn't work)
WIDTH_METERS = 0.30     # Physical plate width [m]
FRAME_WIDTH = 600       # Resize frame width [px]
COLOR_LOWER = (5, 120, 120)    # HSV lower bound (orange/yellow ball)
COLOR_UPPER = (25, 255, 255)   # HSV upper bound

# --- LQR Gains ---
# K = [K_pos, K_vel, K_angle, K_angvel]
# Only K_pos and K_vel are used (no IMU for angle/angular velocity)
K = [-70.71, -64.54, 205.87, 20.32]


# ============================================================
#  INVERSE KINEMATICS
# ============================================================

def get_ik_angle(desired_plate_angle_rad, axis_offset):
    """
    Compute servo angle from desired plate tilt angle.

    Uses exact IK: θ_servo = arcsin((R_plate / R_servo) × sin(α_plate))
    instead of a linear approximation, which fails at steep angles
    due to the high mechanical ratio (13.5 / 5.5).

    Args:
        desired_plate_angle_rad: Target plate tilt [rad], clamped to ±0.35 rad (~±20°)
        axis_offset: Servo neutral angle [deg]

    Returns:
        Servo command angle [deg]
    """
    plate_angle = max(-0.35, min(0.35, desired_plate_angle_rad))
    height = R_PLATE * math.sin(plate_angle)
    ratio = height / R_SERVO
    ratio = max(-1.0, min(1.0, ratio))  # Clamp for arcsin domain
    servo_angle_rad = math.asin(ratio)
    servo_angle_deg = math.degrees(servo_angle_rad)
    return axis_offset + servo_angle_deg


# ============================================================
#  MAIN LOOP
# ============================================================

ser = None
cap = None

try:
    # --- Initialize Serial ---
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)  # Wait for Arduino reset

    # --- Initialize Camera ---
    cap = cv2.VideoCapture(CAM_IDX)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    prev_t = time.time()
    prev_x, prev_y = 0.0, 0.0
    smooth_sx, smooth_sy = OFFSET_X, OFFSET_Y

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # --- Preprocessing ---
        frame = imutils.resize(frame, width=FRAME_WIDTH)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # --- Ball Detection (HSV Color Thresholding) ---
        mask = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        rx, ry = 0.0, 0.0

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), r) = cv2.minEnclosingCircle(c)

            if r > 10:  # Minimum radius filter
                M = cv2.moments(c)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Convert pixel coordinates to meters (origin at frame center)
                screen_cx = frame.shape[1] // 2
                screen_cy = frame.shape[0] // 2
                px_to_m = WIDTH_METERS / frame.shape[1]
                rx = (cx - screen_cx) * px_to_m
                ry = (cy - screen_cy) * px_to_m

                if INVERT_Y_AXIS:
                    ry = -ry

                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        # --- Velocity Estimation (Finite Difference) ---
        curr_t = time.time()
        dt = curr_t - prev_t
        if dt < 0.005:
            dt = 0.005  # Prevent division by near-zero

        vx = (rx - prev_x) / dt
        vy = (ry - prev_y) / dt

        # Reject velocity spikes (noise / detection jumps)
        if abs(vx) > 2.0:
            vx = 0
        if abs(vy) > 2.0:
            vy = 0

        # --- LQR Control Law ---
        # u = -(K1 * position + K2 * velocity) * scaling
        # Only position and velocity gains used (no IMU for angle states)
        ux = -(K[0] * rx + K[1] * vx) * LQR_SCALING
        uy = -(K[0] * ry + K[1] * vy) * LQR_SCALING

        # --- Inverse Kinematics ---
        target_sx = get_ik_angle(ux, OFFSET_X)
        target_sy = get_ik_angle(uy, OFFSET_Y)

        # --- Low-Pass Filter (Anti-Vibration) ---
        smooth_sx = (smooth_sx * SMOOTH_FACTOR) + (target_sx * (1.0 - SMOOTH_FACTOR))
        smooth_sy = (smooth_sy * SMOOTH_FACTOR) + (target_sy * (1.0 - SMOOTH_FACTOR))

        # --- Clamp & Send ---
        sx = int(max(X_MIN, min(X_MAX, smooth_sx)))
        sy = int(max(Y_MIN, min(Y_MAX, smooth_sy)))

        if ser:
            try:
                ser.write(f"SX{sx} SY{sy}\n".encode())
            except Exception:
                pass

        # --- Update State ---
        prev_t = curr_t
        prev_x, prev_y = rx, ry

        # --- Display ---
        cv2.putText(frame, f"Cmd: {sx} | {sy}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Ball & Plate Controller", frame)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    if ser:
        ser.close()
    if cap:
        cap.release()
    cv2.destroyAllWindows()
