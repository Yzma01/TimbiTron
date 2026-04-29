/*
 * BrachioGraph ESP32 Firmware — Serial transport
 *
 * Receives JSON servo commands from the PC over USB Serial (115200 baud).
 * One JSON object per line (terminated by \n).
 *
 * Required libraries (install via Arduino Library Manager):
 *   - ESP32Servo
 *   - ArduinoJson 6.x
 *
 * Command format — one JSON object per line:
 *   {"s1": 1450, "s2": 1300}           → shoulder / elbow pulse-widths (µs)
 *   {"pen": 1200, "pen_ms": 250}       → pen servo with easing over 250 ms
 *   {"s1": 0, "s2": 0, "pen": 0}      → detach all servos
 */

#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Configuration — adjust pins to match your wiring
// ---------------------------------------------------------------------------
const int SERVO1_PIN = 27;   // shoulder
const int SERVO2_PIN = 33;   // elbow
const int SERVO3_PIN = 32;   // pen

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
// ---------------------------------------------------------------------------

Servo servo1, servo2, servo3;

String inputBuffer = "";

// pen easing state
int           pen_current          = 0;
int           pen_target           = 0;
int           pen_step_dir         = 0;
unsigned long pen_step_at_us       = 0;
unsigned long pen_step_interval_us = 1000;

// command counter for monitoring
unsigned long commands_received = 0;

// ---------------------------------------------------------------------------

void applyServo(Servo& servo, int pin, int pw) {
    if (pw == 0) {
        servo.detach();
        Serial.printf("[SERVO] pin %d detached\n", pin);
        return;
    }
    if (pw < SERVO_MIN_US || pw > SERVO_MAX_US) {
        Serial.printf("[SERVO] pin %d REJECTED pw=%d (out of range %d-%d)\n",
                      pin, pw, SERVO_MIN_US, SERVO_MAX_US);
        return;
    }
    if (!servo.attached()) {
        servo.attach(pin, SERVO_MIN_US, SERVO_MAX_US);
        Serial.printf("[SERVO] pin %d attached\n", pin);
    }
    servo.writeMicroseconds(pw);
    Serial.printf("[SERVO] pin %d -> %d us\n", pin, pw);
}

void handleCommand(const String& line) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
        Serial.printf("[JSON] parse error: %s  raw: %s\n", err.c_str(), line.c_str());
        return;
    }

    commands_received++;
    Serial.printf("[CMD] #%lu: %s\n", commands_received, line.c_str());

    if (doc.containsKey("s1")) {
        applyServo(servo1, SERVO1_PIN, doc["s1"].as<int>());
    }
    if (doc.containsKey("s2")) {
        applyServo(servo2, SERVO2_PIN, doc["s2"].as<int>());
    }
    if (doc.containsKey("pen")) {
        int target  = doc["pen"].as<int>();
        int ease_ms = doc["pen_ms"] | 0;

        if (ease_ms == 0 || target == 0 || pen_current == 0) {
            applyServo(servo3, SERVO3_PIN, target);
            pen_current  = target;
            pen_target   = target;
            pen_step_dir = 0;
        } else {
            pen_target   = target;
            pen_step_dir = (target > pen_current) ? 1 : -1;
            long delta   = abs(target - pen_current);
            pen_step_interval_us = ((long)ease_ms * 1000L) / delta;
            pen_step_at_us = micros();
            if (!servo3.attached()) {
                servo3.attach(SERVO3_PIN, SERVO_MIN_US, SERVO_MAX_US);
            }
            Serial.printf("[PEN] easing %d -> %d over %d ms\n",
                          pen_current, target, ease_ms);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n[BrachioGraph ESP32 - Serial transport]");
    Serial.println("Ready. Waiting for JSON commands over USB serial...");
}

void loop() {
    // Read incoming serial data one character at a time; process on newline
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                handleCommand(inputBuffer);
            }
            inputBuffer = "";
        } else if (c != '\r') {
            inputBuffer += c;
        }
    }

    // pen easing tick
    if (pen_step_dir != 0) {
        unsigned long now = micros();
        if (now - pen_step_at_us >= pen_step_interval_us) {
            pen_current    += pen_step_dir;
            servo3.writeMicroseconds(pen_current);
            pen_step_at_us  = now;
            if (pen_current == pen_target) {
                pen_step_dir = 0;
                Serial.printf("[PEN] easing complete at %d us\n", pen_current);
            }
        }
    }
}
