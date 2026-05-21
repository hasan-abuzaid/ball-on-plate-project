/*
 * Ball & Plate — Servo Controller Firmware ("The Muscle")
 * ========================================================
 * Platform:  Arduino Mega 2560
 * Author:    Hasan Abuzaid
 * Course:    Robotics & Control Systems II — GJU
 *
 * This firmware acts as a "dumb listener" — it receives servo angle
 * commands from the host PC over serial and drives two servos with
 * hardcoded safety clamps to prevent the linkage from breaking itself.
 *
 * Serial Protocol:
 *   Baud rate: 115200
 *   Format:    "SX<angle> SY<angle>\n"
 *   Example:   "SX105 SY100\n"  → both servos to calibrated neutral
 *
 * Calibrated Offsets (level plate):
 *   X-axis: 105°
 *   Y-axis: 100°
 *
 * Wiring:
 *   Servo X signal → Pin 9
 *   Servo Y signal → Pin 10
 */

#include <Servo.h>

Servo servoX;
Servo servoY;

// --- Pin Assignments ---
const int PIN_X = 9;
const int PIN_Y = 10;

// --- Calibrated Neutral Positions ---
const int START_X = 105;
const int START_Y = 100;

// --- Safety Limits (prevents mechanical damage) ---
const int LIM_X_MIN = 50;
const int LIM_X_MAX = 150;
const int LIM_Y_MIN = 60;
const int LIM_Y_MAX = 150;

void setup() {
    Serial.begin(115200);

    servoX.attach(PIN_X);
    servoY.attach(PIN_Y);

    // Initialize to calibrated neutral (plate level)
    servoX.write(START_X);
    servoY.write(START_Y);
}

void loop() {
    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();

        // Parse expected format: "SX<angle> SY<angle>"
        if (msg.startsWith("SX")) {
            int idx = msg.indexOf(" SY");

            if (idx > 0) {
                int valX = msg.substring(2, idx).toInt();
                int valY = msg.substring(idx + 3).toInt();

                // Safety clamp to prevent linkage damage
                valX = constrain(valX, LIM_X_MIN, LIM_X_MAX);
                valY = constrain(valY, LIM_Y_MIN, LIM_Y_MAX);

                servoX.write(valX);
                servoY.write(valY);
            }
        }
    }
}
