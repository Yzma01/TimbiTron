/*
 * BrachioGraph — Servo Centering / Calibration
 *
 * Moves all three servos to 1500 µs (mechanical center) once on boot.
 * Use this to verify wiring and find the physical center position of each arm.
 *
 * Required library: ESP32Servo
 *
 * Flash once, open Serial Monitor at 115200 baud, then reset the ESP32.
 * The servos will move to center and hold there.
 */

#include <ESP32Servo.h>

const int SERVO1_PIN = 27;   // shoulder
const int SERVO2_PIN = 33;   // elbow
const int SERVO3_PIN = 32;   // pen

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int CENTER_PW    = 1500;

Servo servo1, servo2, servo3;

void centerServo(Servo& servo, int pin, const char* name) {
    servo.attach(pin, SERVO_MIN_US, SERVO_MAX_US);
    servo.writeMicroseconds(CENTER_PW);
    Serial.printf("%s (pin %d) → %d µs\n", name, pin, CENTER_PW);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n[Servo Calibration — centering all servos to 1500 µs]");

    centerServo(servo1, SERVO1_PIN, "Shoulder (s1)");
    centerServo(servo2, SERVO2_PIN, "Elbow    (s2)");
    centerServo(servo3, SERVO3_PIN, "Pen      (s3)");

    Serial.println("Done. Servos are holding center position.");
    Serial.println("Power off when finished.");
}

void loop() {
    // nothing — servos hold their position
}
