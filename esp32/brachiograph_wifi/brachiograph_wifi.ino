/*
 * BrachioGraph ESP32 firmware — WiFi TCP transport with ACK flow-control
 *
 * Protocol:
 *   One JSON object per line over TCP, exactly like the serial firmware.
 *   Responses are READY / OK / ERR:<reason> and # debug lines.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <WiFi.h>

const char* WIFI_SSID = "yzma";
const char* WIFI_PASSWORD = "susanaselomama27";

const uint16_t SERVER_PORT = 8888;

const int SERVO1_PIN = 4;
const int SERVO2_PIN =5;
const int SERVO3_PIN = 18;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const size_t INPUT_BUF_SIZE = 192;

Servo servo1, servo2, servo3;
WiFiServer server(SERVER_PORT);
WiFiClient client;

char inputBuf[INPUT_BUF_SIZE];
size_t inputLen = 0;

int pen_current = 0;
int pen_target = 0;
int pen_step_dir = 0;
unsigned long pen_step_at_us = 0;
unsigned long pen_step_interval_us = 1000;

static void sendLine(const char* line) {
  if (client && client.connected()) {
    client.println(line);
  }
}

static void sendInfo(const char* line) {
  Serial.println(line);
  sendLine(line);
}

static inline void sendOK() { sendLine("OK"); }

static void sendError(const char* why) {
  if (client && client.connected()) {
    client.print("ERR:");
    client.println(why);
  }
}

static bool applyServo(Servo& servo, int pin, int pw) {
  if (pw == 0) {
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

  if (doc.size() == 0) {
    sendOK();
    return;
  }

  bool has_s1 = doc.containsKey("s1");
  bool has_s2 = doc.containsKey("s2");
  bool has_pen = doc.containsKey("pen");

  int s1 = has_s1 ? doc["s1"].as<int>() : -1;
  int s2 = has_s2 ? doc["s2"].as<int>() : -1;
  int pen = has_pen ? doc["pen"].as<int>() : -1;

  auto inRange = [](int pw) {
    return pw == 0 || (pw >= SERVO_MIN_US && pw <= SERVO_MAX_US);
  };
  if ((has_s1 && !inRange(s1)) || (has_s2 && !inRange(s2)) || (has_pen && !inRange(pen))) {
    sendError("pw_out_of_range");
    return;
  }

  if (has_s1) applyServo(servo1, SERVO1_PIN, s1);
  if (has_s2) applyServo(servo2, SERVO2_PIN, s2);

  if (has_pen) {
    int ease_ms = doc["pen_ms"] | 0;
    if (ease_ms <= 0 || pen == 0 || pen_current == 0) {
      applyServo(servo3, SERVO3_PIN, pen);
      pen_current = pen;
      pen_target = pen;
      pen_step_dir = 0;
    } else {
      pen_target = pen;
      long delta = abs(pen - pen_current);
      if (delta == 0) {
        pen_step_dir = 0;
      } else {
        pen_step_dir = (pen > pen_current) ? 1 : -1;
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

static void ensureClient() {
  if (client && client.connected()) {
    return;
  }

  if (client) {
    client.stop();
  }

  WiFiClient next = server.available();
  if (next) {
    client = next;
    inputLen = 0;
    sendInfo("READY");
    sendInfo("# client connected");
  }
}

void setup() {
  Serial.begin(115200);
  delay(80);
  Serial.println();
  Serial.println("# BrachioGraph ESP32 firmware v2 (wifi + ACK)");
  Serial.print("# connecting to WiFi SSID ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("# WiFi connected, IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("# TCP port: ");
  Serial.println(SERVER_PORT);

  server.begin();
  server.setNoDelay(true);
  Serial.println("# waiting for TCP client");
}

void loop() {
  ensureClient();

  while (client && client.connected() && client.available()) {
    char c = (char)client.read();
    if (c == '\r') continue;
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
      inputLen = 0;
      sendError("line_too_long");
    }
  }

  if (client && !client.connected()) {
    client.stop();
  }

  if (pen_step_dir != 0) {
    unsigned long now = micros();
    if (now - pen_step_at_us >= pen_step_interval_us) {
      pen_current += pen_step_dir;
      servo3.writeMicroseconds(pen_current);
      pen_step_at_us = now;
      if (pen_current == pen_target) pen_step_dir = 0;
    }
  }
}
