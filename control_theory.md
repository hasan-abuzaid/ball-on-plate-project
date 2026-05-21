# Control Theory — Plant Model, LQR Design & Stability

## 1. System Description

A ball rolls freely on a 30×30 cm plate that can tilt about two axes (X and Y), each driven by a servo motor. The control objective is to stabilize the ball at a desired position with settling time < 3 seconds and overshoot < 5%.

The system is modeled as **two independent ball-and-beam systems**. This decoupling assumption holds because inter-axis coupling is negligible at small plate angles.

## 2. Physical Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Ball mass | m | 0.002 kg |
| Ball radius | R | 0.04 m |
| Gravity | g | 9.81 m/s² |
| Plate length | L | 0.30 m |
| Servo arm length | d | 0.055 m |
| Linkage rod length | — | 0.11 m |
| Center-to-rod distance | — | 0.13–0.14 m |

## 3. Dynamic Modeling

For a ball rolling without slipping on an inclined surface, the equation of motion (linearized around α = 0) is:

```
(m + J/R²) · r̈ = -m · g · sin(α)  ≈  -m · g · α    (small angle)
```

Ball inertia (solid sphere): `J = (2/5)mR²`

The coupling constant:
```
H = -mg / (J/R² + m)
```

This gives: `r̈ = H · α`

## 4. State-Space Representation

**State vector:**
```
x = [r, ṙ, α, α̇]ᵀ
    = [position, velocity, plate_angle, angular_velocity]ᵀ
```

**System matrices:**
```
A = | 0  1  0  0 |     B = | 0 |
    | 0  0  H  0 |         | 0 |
    | 0  0  0  1 |         | 0 |
    | 0  0  0  0 |         | 1 |

C = [1  0  0  0]         D = [0]
```

## 5. Controllability

The controllability matrix `Co = [B, AB, A²B, A³B]` has rank 4 (equal to the number of states). The system is **fully controllable**, confirming that full-state feedback can stabilize it.

## 6. Open-Loop Stability

Eigenvalues of A: `λ = {0, 0, 0, 0}`

Four poles at the origin → **marginally unstable**. Any perturbation causes the ball to accelerate off the plate.

## 7. Controller Selection

Pole placement (Ackermann) was considered but rejected in favor of **LQR** because:
- LQR provides optimality guarantees (minimizes a cost function)
- Produces smoother servo commands
- More robust to model uncertainty
- Better real-world hardware performance

## 8. LQR Design

### Cost Function

```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

### Tuning Matrices (Project Values)

```
Q = diag([300, 20, 150, 2])
R = 1
```

| Weight | Meaning |
|---|---|
| Q(1,1) = 300 | Aggressive position error penalty |
| Q(2,2) = 20 | Velocity damping |
| Q(3,3) = 150 | Angle tracking |
| Q(4,4) = 2 | Angular rate damping |
| R = 1 | Moderate control effort penalty |

The MATLAB `lqr()` function solves the algebraic Riccati equation to produce optimal gains `K`.

### Control Law

```
u = -Kx + N̄r
```

where `N̄` is the reference scaling gain computed via `rscale()` to ensure zero steady-state error.

## 9. Closed-Loop Stability

With state feedback, the closed-loop dynamics are:

```
ẋ = (A - BK)x
```

All eigenvalues of `(A - BK)` have **negative real parts** → **asymptotically stable**.

## 10. Implementation Note

In the physical system, only position and velocity are measured (via camera). The controller uses only `K₁` (position) and `K₂` (velocity) gains:

```
u = -(K₁ · position + K₂ · velocity)
```

The plate angle and angular velocity states are not measured because there is no IMU. This works because the servo dynamics are fast relative to the ball dynamics, so the inner loop effectively self-regulates.

## 11. Performance

| Metric | Requirement | Achieved |
|---|---|---|
| Settling time | < 3 s | ~2–3 s |
| Overshoot | < 5% | < 5% |
| Steady-state error | Zero | Zero (with N̄) |

## References

- Ogata, K. *Modern Control Engineering*, 5th Edition
- Franklin, Powell, Emami-Naeini. *Feedback Control of Dynamic Systems*
