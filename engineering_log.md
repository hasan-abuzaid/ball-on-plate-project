# Engineering Log — Problems & Solutions

A record of the critical engineering hurdles encountered during development and how each was resolved. This is the real story of the project — the math was the easy part.

---

## 1. The Vibration Problem

**Symptom:** Servos slammed back and forth violently, making the plate shake instead of stabilize.

**Root Cause:** The raw LQR output changed significantly between control loop iterations. Even small noise in the ball position measurement caused large instantaneous changes in the servo command. The servos, being fast mechanical actuators, faithfully executed every jittery command.

**Solution:** Implemented a first-order low-pass filter on the servo commands:

```
smooth_value = (old_value × 0.6) + (new_target × 0.4)
```

This acts as an electronic shock absorber — the servo command changes gradually instead of jumping. The 0.6/0.4 split was tuned empirically: too much smoothing (e.g., 0.9/0.1) makes the system sluggish; too little (e.g., 0.3/0.7) lets vibration through.

---

## 2. The Y-Axis Runaway

**Symptom:** X-axis worked perfectly. Y-axis pushed the ball off the plate instead of catching it — the harder the controller tried, the faster the ball fell.

**Root Cause:** The Y-axis servo was physically mounted with opposite orientation to what the math assumed. A positive control signal that should have tilted the plate "left" was actually tilting it "right."

**Solution:** Applied a −1.0 scaling factor to the Y-axis control output. Alternatively, `INVERT_Y_AXIS = True` in the configuration flips the camera Y coordinate instead.

**Lesson:** Always test each axis independently before running the full 2-DOF controller. A sign error in one axis turns a stabilizing controller into a destabilizing one.

---

## 3. The Edge "Lazy" Phenomenon

**Symptom:** The ball balanced fine near the center but behaved sluggishly near the plate edges. It would drift off slowly as if the controller wasn't trying hard enough.

**Root Cause:** The initial implementation used a linear approximation to map plate angles to servo angles:

```
θ_servo ≈ (R_plate / R_servo) × α_plate
```

This works for small angles but breaks down at larger tilts because `sin(θ) ≠ θ` when θ is significant. With a mechanical ratio of 13.5/5.5 ≈ 2.45, the nonlinearity kicks in within the normal operating range.

**Solution:** Replaced the linear map with exact inverse kinematics:

```
θ_servo = arcsin( (R_plate / R_servo) × sin(α_plate) )
```

This correctly handles the full nonlinear geometry of the servo-linkage-plate mechanism.

---

## 4. Port Locking & Circular Import Crashes

**Symptom:** Python scripts crashed on startup with serial port errors or mysterious import failures.

**Root Cause (Port Locking):** Spyder IDE's IPython kernel held the COM3 serial connection open even after the script terminated. Restarting the script tried to open an already-locked port.

**Root Cause (Circular Import):** A Python file was accidentally named `serial.py`. When the script executed `import serial`, Python imported this file instead of the `pyserial` library, creating a circular import that crashed instantly.

**Solution:**
- Renamed the conflicting file (never name a file after a library you import)
- Added a `try/finally` block that guarantees `ser.close()` runs on exit
- Used Spyder's "Restart Kernel" before re-running after serial errors

---

## 5. Camera Coordinate System Mismatch

**Symptom:** Ball tracking worked but the controller responded incorrectly to ball movement in certain directions.

**Root Cause:** OpenCV's pixel coordinate system has (0,0) at the top-left with Y increasing downward. The physical plate coordinate system has Y increasing upward. Without accounting for this, the controller's Y-axis response was inverted.

**Solution:** The pixel-to-meter conversion centers the coordinate system at the frame midpoint, and the `INVERT_Y_AXIS` flag handles the direction flip if needed for the specific camera mounting orientation.
