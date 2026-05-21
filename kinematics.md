# Kinematics — Forward & Inverse

## 1. Mechanism Overview

The ball-on-plate platform is a 2-DOF tilting mechanism. Each axis uses a **servo motor** connected to the plate edge via a **rigid linkage rod**. The servo horn and plate arm form a 4-bar-like linkage where the servo angle determines the plate tilt.

### Geometry

```
            ← R_plate →
    ────────────────────── Plate surface
    │                    │
    │       Pivot        │
    │      (center)      │
    │                    │
    └────────┬───────────┘
             │ Linkage rod (L_rod)
             │
    ┌────────┘
    │ Servo horn (R_servo)
    ●─── Servo axis
```

| Parameter | Symbol | Value |
|---|---|---|
| Plate arm radius | R_plate | 13.5 cm |
| Servo horn length | R_servo | 5.5 cm |
| Linkage rod length | L_rod | 11.0 cm |
| Center-to-rod distance | — | 13.0–14.0 cm |

## 2. Why Linear Approximation Fails

For small angles, the servo-to-plate mapping appears approximately linear:

```
θ_servo ≈ (R_plate / R_servo) × α_plate
```

But the **mechanical ratio** `R_plate / R_servo = 13.5 / 5.5 ≈ 2.45` is too large for this to hold across the operating range. At steeper angles, the linear approximation underestimates the required servo deflection, causing the ball to behave "lazily" near the plate edges — it won't tilt enough to catch the ball.

This was the **"Edge Lazy" phenomenon** observed during testing.

## 3. Exact Inverse Kinematics

### Derivation

The plate tilts by angle `α_plate`, raising one edge by height `h`:

```
h = R_plate × sin(α_plate)
```

The servo horn must produce this same height change. The servo angle `θ_servo` that achieves height `h` with a horn of length `R_servo`:

```
h = R_servo × sin(θ_servo)
```

Setting equal:

```
R_servo × sin(θ_servo) = R_plate × sin(α_plate)
```

Solving for the servo angle:

```
θ_servo = arcsin( (R_plate / R_servo) × sin(α_plate) )
```

### Implementation

The final servo command adds the axis-specific offset (neutral position):

```python
def get_ik_angle(desired_plate_angle_rad, axis_offset):
    plate_angle = clamp(desired_plate_angle_rad, -0.35, 0.35)  # ±20° limit
    height = R_PLATE * sin(plate_angle)
    ratio = clamp(height / R_SERVO, -1.0, 1.0)                 # arcsin domain
    servo_angle_deg = degrees(asin(ratio))
    return axis_offset + servo_angle_deg
```

The ±0.35 rad (~±20°) clamp prevents the `arcsin` argument from exceeding ±1.0, which would cause a math domain error.

## 4. Forward Kinematics

Given a servo angle, the resulting plate tilt is:

```
α_plate = arcsin( (R_servo / R_plate) × sin(θ_servo - θ_offset) )
```

This is used implicitly during calibration (IK calibrator tool) to verify that a commanded servo angle produces the expected physical plate tilt.

## 5. Calibration

The `python/ik_calibrator.py` tool lets you verify the geometry without running the full vision system:

1. Input a desired plate angle in degrees
2. The tool computes the servo angle via IK
3. Sends the command to Arduino
4. You physically verify the plate tilt matches

### Calibrated Offsets

| Axis | Neutral (Level Plate) |
|---|---|
| X | 105° |
| Y | 100° |

These offsets account for mechanical asymmetry in the build — the servos don't produce a level plate at exactly 90°.
