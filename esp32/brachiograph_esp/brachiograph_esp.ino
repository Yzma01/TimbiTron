/*
 * BrachioGraph ESP32 firmware — Serial transport with ACK flow-control
 *
 * Protocol  (USB Serial, 115200 8N1, LF line-terminator)
 * --------------------------------------------------------------------------
 *   PC  -> ESP32 :  one JSON object per line, ended with '\n'
 *       {"s1": 1450, "s2": 1300}       set shoulder/elbow pulse-widths (µs)
 *       {"pen": 1200, "pen_ms": 250}   move pen servo, easing 250 ms
 *       {"s1": 0, "s2": 0, "pen": 0}   detach (de-energise) all servos
 *       {}                             ping — always replies OK
 *
 *   ESP32 -> PC  :  one response per command, ended with '\n'
 *       READY                          one-time banner right after boot
 *       OK                             command parsed & applied
 *       ERR:<short_reason>             command rejected (the PC logs it)
 *       # <anything>                   informational; PC ignores '#' lines
 *
 * Why this design works where WebSocket / TCP / UDP / raw Serial don't
 * --------------------------------------------------------------------------
 *   - explicit framing: every message is exactly one line → no partial reads
 *   - ACK per command: the PC never sends a second command until the first
 *     one is acknowledged, so the UART RX buffer can't overflow
 *   - READY banner + {} ping: the PC knows *exactly* when the ESP32 is ready
 *     to talk, regardless of whether it just booted or was already running
 *   - no Arduino String: a fixed char buffer avoids heap fragmentation over
 *     long plots (thousands of commands)
 *
 * Required libraries (Arduino IDE -> Library Manager)
 *   - ESP32Servo       (Kevin Harrington)
 *   - ArduinoJson 6.x  (Benoît Blanchon)   — v7 renames the API
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Pin map — edit to match your wiring
//   AVOID strapping pins on the ESP32: 0, 2, 5, 12, 15.
//   AVOID input-only pins 34, 35, 36, 39 (they can't drive a servo line).
// ---------------------------------------------------------------------------
const int SERVO1_PIN = 4;   // shoulder
const int SERVO2_PIN = 5;   // elbow
const int SERVO3_PIN = 32;   // pen

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Commands are small JSON (~40 bytes). 192 leaves plenty of margin.
const size_t INPUT_BUF_SIZE = 192;

// ---------------------------------------------------------------------------

Servo servo1, servo2, servo3;

char   inputBuf[INPUT_BUF_SIZE];
size_t inputLen = 0;

// Pen-easing state
int           pen_current          = 0;    // last-written pen PW (0 = detached)
int           pen_target           = 0;
int           pen_step_dir         = 0;    // +1, -1, or 0 when idle
unsigned long pen_step_at_us       = 0;
unsigned long pen_step_interval_us = 1000;

// ---------------------------------------------------------------------------

static inline void sendOK()                     { Serial.println("OK"); }
static inline void sendError(const char* why)   { Serial.print("ERR:"); Serial.println(why); }

/*
 * applyServo() returns false if the pulse-width is out of range;
 * in that case the servo is left untouched and the caller sends ERR back.
 */
static bool applyServo(Servo& servo, int pin, int pw) {
    if (pw == 0) {                         // 0 means "detach / de-energise"
        if (servo.attached()) servo.detach();
        return true;
    }
    if (pw < SERVO_MIN_US || pw > SERVO_MAX_US) {
        return false;
    }
    if (!servo.attached()) {
        servo.attach(pin, SERVO_MIN_US, SERVO_MAX_US);
    }
    servo.writeMicroseconds(pw);
    return true;
}

static void handleLine(const char* line, size_t len) {
    StaticJsonDocument<192> doc;
    DeserializationError err = deserializeJson(doc, line, len);
    if (err) {
        sendError(err.c_str());
        return;
    }

    // {} is a valid "ping"; just ACK it.
    if (doc.size() == 0) { sendOK(); return; }

    // Apply each field that's present. First range-check everything so we
    // never leave the arm half-moved after rejecting one of the two values.
    bool has_s1  = doc.containsKey("s1");
    bool has_s2  = doc.containsKey("s2");
    bool has_pen = doc.containsKey("pen");

    int s1  = has_s1  ? doc["s1"].as<int>()  : -1;
    int s2  = has_s2  ? doc["s2"].as<int>()  : -1;
    int pen = has_pen ? doc["pen"].as<int>() : -1;

    auto inRange = [](int pw) {
        return pw == 0 || (pw >= SERVO_MIN_US && pw <= SERVO_MAX_US);
    };
    if ((has_s1  && !inRange(s1)) ||
        (has_s2  && !inRange(s2)) ||
        (has_pen && !inRange(pen))) {
        sendError("pw_out_of_range");
        return;
    }

    if (has_s1) applyServo(servo1, SERVO1_PIN, s1);
    if (has_s2) applyServo(servo2, SERVO2_PIN, s2);

    if (has_pen) {
        int ease_ms = doc["pen_ms"] | 0;

        if (ease_ms <= 0 || pen == 0 || pen_current == 0) {
            // snap immediately (no interpolation possible from a detached state)
            applyServo(servo3, SERVO3_PIN, pen);
            pen_current  = pen;
            pen_target   = pen;
            pen_step_dir = 0;
        } else {
            // ease: step 1 µs at a time over ease_ms
            pen_target   = pen;
            long delta   = abs(pen - pen_current);
            if (delta == 0) {
                pen_step_dir = 0;
            } else {
                pen_step_dir         = (pen > pen_current) ? 1 : -1;
                pen_step_interval_us = ((long)ease_ms * 1000L) / delta;
                if (pen_step_interval_us == 0) pen_step_interval_us = 1;
                pen_step_at_us = micros();
                if (!servo3.attached()) {
                    servo3.attach(SERVO3_PIN, SERVO_MIN_US, SERVO_MAX_US);
                }
            }
        }
    }

    sendOK();
}

void setup() {
    Serial.begin(115200);
    // Give the USB-UART bridge a moment to settle after a reset; otherwise our
    // banner can get eaten along with the bootloader noise.
    delay(80);
    Serial.println();
    Serial.println("# BrachioGraph ESP32 firmware v2 (serial + ACK)");
    Serial.print  ("# pins s1=");  Serial.print(SERVO1_PIN);
    Serial.print  (" s2=");        Serial.print(SERVO2_PIN);
    Serial.print  (" pen=");       Serial.println(SERVO3_PIN);
    Serial.print  ("# pw range ");
    Serial.print  (SERVO_MIN_US);  Serial.print("..");
    Serial.println(SERVO_MAX_US);
    Serial.println("READY");
}

void loop() {
    // ---- drain incoming bytes, framing on '\n' ----------------------------
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;                       // ignore CR
        if (c == '\n') {
            if (inputLen > 0) {
                inputBuf[inputLen] = '\0';
                handleLine(inputBuf, inputLen);
                inputLen = 0;
            }
            continue;
        }
        if (inputLen < INPUT_BUF_SIZE - 1) {
            inputBuf[inputLen++] = c;
        } else {
            inputLen = 0;                              // discard the line
            sendError("line_too_long");
        }
    }

    // ---- pen easing tick --------------------------------------------------
    if (pen_step_dir != 0) {
        unsigned long now = micros();
        if (now - pen_step_at_us >= pen_step_interval_us) {
            pen_current    += pen_step_dir;
            servo3.writeMicroseconds(pen_current);
            pen_step_at_us  = now;
            if (pen_current == pen_target) pen_step_dir = 0;
        }
    }
}
