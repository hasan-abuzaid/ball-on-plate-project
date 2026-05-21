# Hardware — Bill of Materials & Wiring

## Bill of Materials

| # | Component | Specification | Qty |
|---|---|---|---|
| 1 | Plate | 30×30 cm, flat acrylic or MDF | 1 |
| 2 | Ping pong ball | 40 mm diameter, ~2 g | 1 |
| 3 | Servo motors | SG90 or MG996R | 2 |
| 4 | Servo horns | 5.5 cm arm length | 2 |
| 5 | Linkage rods | 11 cm, rigid (metal or plastic) | 2 |
| 6 | Universal joint | Ball joint or gimbal at plate center | 1 |
| 7 | Arduino Mega 2560 | ATmega2560, USB serial | 1 |
| 8 | USB webcam | 640×480 minimum resolution | 1 |
| 9 | Camera mount | Positioned directly above plate center | 1 |
| 10 | Frame / base | Rigid structure (wood, acrylic, aluminum) | 1 |
| 11 | USB cable | Type-B to Type-A (Arduino to PC) | 1 |
| 12 | Power supply | 5–6V, 2A+ (external for high-torque servos) | 1 |
| 13 | Jumper wires | Male-to-male | ~10 |

## Wiring

```
                    ┌──────────────────┐
                    │   Arduino Mega   │
                    │                  │
  Servo X Signal ──→│ D9               │
  Servo Y Signal ──→│ D10              │
                    │                  │
                    │ USB ─────────────│──→ Host PC (Python controller)
                    │                  │
                    │ GND ─────────────│──→ Common GND
                    └──────────────────┘

  External Power (recommended for MG996R):
  ┌─────────────┐
  │ 6V / 2A PSU │──→ Servo VCC (both servos)
  │             │──→ Servo GND ──→ Arduino GND (common ground)
  └─────────────┘
```

## Critical Dimensions

```
    ←─── 30 cm ───→
    ┌──────────────┐
    │              │
    │    Pivot     │ ← Universal joint at center
    │     (●)      │
    │      │       │
    │      │ 13-14 cm from center to rod attachment
    │      │       │
    └──────┼───────┘
           │
           │ Linkage rod: 11 cm
           │
    ┌──────┘
    │ Servo horn: 5.5 cm
    ●── Servo motor
```

## Assembly Notes

1. The universal joint must be at the **exact center** of the plate — any offset creates asymmetric behavior between axes
2. Linkage rods must be **rigid** — any flex introduces unmodeled dynamics
3. Mount the camera **directly above** the plate center, high enough to see the full 30×30 cm surface
4. Ensure the frame doesn't flex — structural rigidity matters for control accuracy
5. The servo neutral positions are **not** 90° — they're calibrated to 105° (X) and 100° (Y) to account for mechanical asymmetry in the build
