#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <WiFiNINA.h>

#include <cstring>

#include "secrets.h"

// --- Pin assignments ---
Servo leftEyeX;   // horizontal
Servo leftEyeY;   // vertical
Servo rightEyeX;  // horizontal
Servo rightEyeY;  // vertical
constexpr uint8_t LEFT_EYE_X_PIN = 20;
constexpr uint8_t LEFT_EYE_Y_PIN = 2;
constexpr uint8_t RIGHT_EYE_X_PIN = 21;
constexpr uint8_t RIGHT_EYE_Y_PIN = 3;
constexpr uint8_t BUZZER_PIN = 12;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// --- Timing ---
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
constexpr uint32_t MQTT_RETRY_INTERVAL_MS = 5000;
constexpr uint32_t FACE_STATE_SOUND_DELAY_MS = 10000;
constexpr uint32_t CONTROL_MODE_TIMEOUT_MS = 10000;

// --- Camera input range ---
constexpr int CAMERA_X_MIN = 0;
constexpr int CAMERA_X_MAX = 640;
constexpr int CAMERA_Y_MIN = 0;
constexpr int CAMERA_Y_MAX = 480;

// --- Servo angle limits (SG90) ---
constexpr int SERVO_X_MIN_ANGLE = 60;
constexpr int SERVO_X_MAX_ANGLE = 120;
constexpr int SERVO_Y_MIN_ANGLE = 60;
constexpr int SERVO_Y_MAX_ANGLE = 140;

// --- Movement tuning ---
constexpr int SERVO_X_JITTER_DEADBAND_DEGREES = 2;
constexpr int SERVO_Y_JITTER_DEADBAND_DEGREES = 1;
constexpr int SERVO_Y_MAX_STEP_DEGREES = 6;
constexpr int CONTROL_SERVO_STEP_DEGREES = 8;
constexpr int TEST_SWEEP_STEP_DEGREES = 5;
constexpr uint32_t TEST_SWEEP_STEP_DELAY_MS = 20;

// --- State ---
uint32_t lastWiFiAttemptMs = 0;
uint32_t lastMqttAttemptMs = 0;
int lastXAngle = 90;
int lastYAngle = 90;
uint32_t lastFaceActiveMs = 0;
bool faceActive = false;
bool hasSeenFace = false;
bool sadQueuedForCurrentAbsence = false;
bool happySoundPending = false;
bool sadSoundPending = false;
bool controlModeActive = false;
uint32_t lastControlEventMs = 0;

// --- Helpers ---

void writeAllX(int angle) {
  leftEyeX.write(angle);
  rightEyeX.write(angle);
}

void writeAllY(int angle) {
  leftEyeY.write(angle);
  rightEyeY.write(angle);
}

bool isControlTopic(const char* topic) {
  return strcmp(topic, MQTT_CONTROL_TOPIC) == 0;
}

bool isTrackingTopic(const char* topic) {
  return strcmp(topic, MQTT_TRACKING_TOPIC) == 0;
}

bool payloadEquals(uint8_t* payload, unsigned int length,
                   const char* expected) {
  size_t expectedLength = strlen(expected);
  return (length == expectedLength) &&
         (memcmp(payload, expected, expectedLength) == 0);
}

bool isRecognizedControlEvent(uint8_t* payload, unsigned int length) {
  return payloadEquals(payload, length, "enc1-nurbo-right") ||
         payloadEquals(payload, length, "enc1-nurbo-left") ||
         payloadEquals(payload, length, "enc2-nurbo-right") ||
         payloadEquals(payload, length, "enc2-nurbo-left");
}

int applyDeltaAndConstrain(int currentAngle, int deltaDegrees, int minAngle,
                           int maxAngle) {
  return constrain(currentAngle + deltaDegrees, minAngle, maxAngle);
}

// --- Buzzer ---

void beep(uint16_t freqHz, uint16_t durationMs) {
  tone(BUZZER_PIN, freqHz, durationMs);
  delay(durationMs + 30);
  noTone(BUZZER_PIN);
}

void playBeepCount(int count) {
  for (int i = 0; i < count; ++i) {
    beep(2000, 50);
  }
}

void playHappySound() {
  beep(1200, 80);
  beep(1600, 80);
  beep(2000, 120);
}

void playSadSound() {
  beep(800, 150);
  beep(500, 200);
}

// --- Servo test ---

void sweepXAcrossRange(int minAngle, int maxAngle) {
  for (int angle = minAngle; angle <= maxAngle;
       angle += TEST_SWEEP_STEP_DEGREES) {
    writeAllX(angle);
    lastXAngle = angle;
    delay(TEST_SWEEP_STEP_DELAY_MS);
  }

  for (int angle = maxAngle; angle >= minAngle;
       angle -= TEST_SWEEP_STEP_DEGREES) {
    writeAllX(angle);
    lastXAngle = angle;
    delay(TEST_SWEEP_STEP_DELAY_MS);
  }
}

void sweepYAcrossRange(int minAngle, int maxAngle) {
  for (int angle = minAngle; angle <= maxAngle;
       angle += TEST_SWEEP_STEP_DEGREES) {
    writeAllY(angle);
    lastYAngle = angle;
    delay(TEST_SWEEP_STEP_DELAY_MS);
  }

  for (int angle = maxAngle; angle >= minAngle;
       angle -= TEST_SWEEP_STEP_DEGREES) {
    writeAllY(angle);
    lastYAngle = angle;
    delay(TEST_SWEEP_STEP_DELAY_MS);
  }
}

void sweepAllAcrossRange(int minX, int maxX, int minY, int maxY) {
  int spanX = maxX - minX;
  int spanY = maxY - minY;
  int sharedSpan = max(spanX, spanY);

  for (int p = 0; p <= sharedSpan; p += TEST_SWEEP_STEP_DEGREES) {
    int ax = minX + ((p * spanX) / sharedSpan);
    int ay = minY + ((p * spanY) / sharedSpan);
    writeAllX(ax);
    writeAllY(ay);
    lastXAngle = ax;
    lastYAngle = ay;
    delay(TEST_SWEEP_STEP_DELAY_MS);
  }

  for (int p = sharedSpan; p >= 0; p -= TEST_SWEEP_STEP_DEGREES) {
    int ax = minX + ((p * spanX) / sharedSpan);
    int ay = minY + ((p * spanY) / sharedSpan);
    writeAllX(ax);
    writeAllY(ay);
    lastXAngle = ax;
    lastYAngle = ay;
    delay(TEST_SWEEP_STEP_DELAY_MS);
  }
}

void runServoTestSequence() {
  Serial.println("Starting servo test sequence (X, Y, synchronized)");
  playBeepCount(2);

  Serial.println("Test stage 1: X pair sweep");
  sweepXAcrossRange(SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);
  playBeepCount(1);

  Serial.println("Test stage 2: Y pair sweep");
  sweepYAcrossRange(SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);
  playBeepCount(1);

  Serial.println("Test stage 3: synchronized sweep all servos");
  sweepAllAcrossRange(SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE, SERVO_Y_MIN_ANGLE,
                      SERVO_Y_MAX_ANGLE);

  lastXAngle = 90;
  lastYAngle = 90;
  writeAllX(lastXAngle);
  writeAllY(lastYAngle);

  playHappySound();
  Serial.println("Servo test sequence complete (returned to 90)");
}

// --- Control handling ---

void handleControlEvent(uint8_t* payload, unsigned int length) {
  JsonDocument controlDoc;
  DeserializationError controlParseError =
      deserializeJson(controlDoc, payload, length);

  if (!controlParseError) {
    bool pressed = controlDoc["pressed"] | false;
    int key = controlDoc["key"] | -1;

    if (pressed && key == 15) {
      controlModeActive = false;
      Serial.println("Exited control-mode state (key 15 override)");
      return;
    }

    if (pressed && key == 14) {
      if (!controlModeActive) {
        controlModeActive = true;
        Serial.println("Entered control-mode state");
      }

      lastControlEventMs = millis();
      Serial.println("Control command: key 14 test sequence");
      runServoTestSequence();
      controlModeActive = false;
      Serial.println("Exited control-mode state (test complete)");
      return;
    }
  }

  if (!isRecognizedControlEvent(payload, length)) {
    Serial.println("Ignoring unrecognized control event");
    return;
  }

  if (!controlModeActive) {
    controlModeActive = true;
    Serial.println("Entered control-mode state");
  }

  lastControlEventMs = millis();

  Serial.print("Control event: ");
  for (unsigned int i = 0; i < length; ++i) {
    Serial.print(static_cast<char>(payload[i]));
  }
  Serial.println();

  if (payloadEquals(payload, length, "enc1-nurbo-right")) {
    lastXAngle = applyDeltaAndConstrain(lastXAngle, CONTROL_SERVO_STEP_DEGREES,
                                        SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);
    writeAllX(lastXAngle);
    Serial.print("Control move -> X: ");
    Serial.println(lastXAngle);
    return;
  }

  if (payloadEquals(payload, length, "enc1-nurbo-left")) {
    lastXAngle = applyDeltaAndConstrain(lastXAngle, -CONTROL_SERVO_STEP_DEGREES,
                                        SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);
    writeAllX(lastXAngle);
    Serial.print("Control move -> X: ");
    Serial.println(lastXAngle);
    return;
  }

  if (payloadEquals(payload, length, "enc2-nurbo-right")) {
    lastYAngle = applyDeltaAndConstrain(lastYAngle, CONTROL_SERVO_STEP_DEGREES,
                                        SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);
    writeAllY(lastYAngle);
    Serial.print("Control move -> Y: ");
    Serial.println(lastYAngle);
    return;
  }

  if (payloadEquals(payload, length, "enc2-nurbo-left")) {
    lastYAngle = applyDeltaAndConstrain(lastYAngle, -CONTROL_SERVO_STEP_DEGREES,
                                        SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);
    writeAllY(lastYAngle);
    Serial.print("Control move -> Y: ");
    Serial.println(lastYAngle);
  }
}

// --- Tracking ---

int mapTrackingXToServo(int x) {
  int cx = constrain(x, CAMERA_X_MIN, CAMERA_X_MAX);
  return map(cx, CAMERA_X_MIN, CAMERA_X_MAX, SERVO_X_MIN_ANGLE,
             SERVO_X_MAX_ANGLE);
}

int mapTrackingYToServo(int y) {
  int cy = constrain(y, CAMERA_Y_MIN, CAMERA_Y_MAX);
  return map(cy, CAMERA_Y_MIN, CAMERA_Y_MAX, SERVO_Y_MIN_ANGLE,
             SERVO_Y_MAX_ANGLE);
}

int applyMaxStep(int currentAngle, int targetAngle, int maxStepDegrees) {
  int delta = targetAngle - currentAngle;

  if (abs(delta) <= maxStepDegrees) {
    return targetAngle;
  }

  return currentAngle + ((delta > 0) ? maxStepDegrees : -maxStepDegrees);
}

void onMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
  if (isControlTopic(topic)) {
    handleControlEvent(payload, length);
    return;
  }

  if (controlModeActive) {
    Serial.println("Ignoring non-control MQTT message while in control mode");
    return;
  }

  if (!isTrackingTopic(topic)) {
    Serial.println("Ignoring MQTT message on unknown topic");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("MQTT JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  bool active = doc["active"] | false;
  int x = doc["x"] | (CAMERA_X_MAX / 2);
  int y = doc["y"] | (CAMERA_Y_MAX / 2);
  bool hasNumericY = doc["y"].is<int>();

  Serial.print("Tracking: x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.print(y);
  Serial.print(" active=");
  Serial.println(active ? "true" : "false");

  uint32_t now = millis();

  if (active) {
    bool wasAbsentLongEnough = (!faceActive) && ((now - lastFaceActiveMs) >=
                                                 FACE_STATE_SOUND_DELAY_MS);

    if (wasAbsentLongEnough) {
      happySoundPending = true;
    }

    faceActive = true;
    hasSeenFace = true;
    sadQueuedForCurrentAbsence = false;
    lastFaceActiveMs = now;
  } else {
    faceActive = false;
  }

  if (!active) {
    Serial.println("No servo update: tracking inactive");
    return;
  }

  int servoXAngle = mapTrackingXToServo(x);
  int servoYAngle = lastYAngle;
  bool wroteX = false;
  bool wroteY = false;

  if (hasNumericY) {
    int targetY = mapTrackingYToServo(y);
    servoYAngle = applyMaxStep(lastYAngle, targetY, SERVO_Y_MAX_STEP_DEGREES);
  }

  if (abs(servoXAngle - lastXAngle) >= SERVO_X_JITTER_DEADBAND_DEGREES) {
    writeAllX(servoXAngle);
    lastXAngle = servoXAngle;
    wroteX = true;
  }

  if (abs(servoYAngle - lastYAngle) >= SERVO_Y_JITTER_DEADBAND_DEGREES) {
    writeAllY(servoYAngle);
    lastYAngle = servoYAngle;
    wroteY = true;
  }

  Serial.print("X: ");
  Serial.print(servoXAngle);
  Serial.print(wroteX ? " (write)" : " (hold)");
  Serial.print(" | Y: ");
  Serial.print(servoYAngle);
  Serial.println(wroteY ? " (write)" : " (hold)");
}

// --- WiFi / MQTT ---

void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  uint32_t now = millis();
  if (now - lastWiFiAttemptMs < WIFI_RETRY_INTERVAL_MS) {
    return;
  }
  lastWiFiAttemptMs = now;

  Serial.print("Connecting to Wi-Fi SSID: ");
  Serial.println(WIFI_SSID);

  const uint32_t startMs = millis();
  WiFi.disconnect();
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED &&
         (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    return;
  }

  Serial.print("Wi-Fi failed, status=");
  Serial.println(WiFi.status());
}

void connectToMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqttClient.connected()) {
    return;
  }

  uint32_t now = millis();
  if (now - lastMqttAttemptMs < MQTT_RETRY_INTERVAL_MS) {
    return;
  }
  lastMqttAttemptMs = now;

  Serial.print("Connecting to MQTT broker: ");
  Serial.print(MQTT_BROKER);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  bool connected = mqttClient.connect(MQTT_CLIENT_ID);

  if (connected) {
    Serial.println("MQTT connected");

    if (mqttClient.subscribe(MQTT_TRACKING_TOPIC)) {
      Serial.print("Subscribed to ");
      Serial.println(MQTT_TRACKING_TOPIC);
    }

    if (mqttClient.subscribe(MQTT_CONTROL_TOPIC)) {
      Serial.print("Subscribed to ");
      Serial.println(MQTT_CONTROL_TOPIC);
    }

    return;
  }

  Serial.print("MQTT connect failed, rc=");
  Serial.println(mqttClient.state());
}

// --- Setup / Loop ---

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(BUZZER_PIN, OUTPUT);

  uint8_t leftEyeXAttachResult = leftEyeX.attach(LEFT_EYE_X_PIN);
  uint8_t leftEyeYAttachResult = leftEyeY.attach(LEFT_EYE_Y_PIN);
  uint8_t rightEyeXAttachResult = rightEyeX.attach(RIGHT_EYE_X_PIN);
  uint8_t rightEyeYAttachResult = rightEyeY.attach(RIGHT_EYE_Y_PIN);

  writeAllX(lastXAngle);
  writeAllY(lastYAngle);
  Serial.print("Left X attach result: ");
  Serial.print(leftEyeXAttachResult);
  Serial.print(" attached=");
  Serial.println(leftEyeX.attached() ? "true" : "false");
  Serial.print("Right X attach result: ");
  Serial.print(rightEyeXAttachResult);
  Serial.print(" attached=");
  Serial.println(rightEyeX.attached() ? "true" : "false");
  Serial.print("Left Y attach result: ");
  Serial.print(leftEyeYAttachResult);
  Serial.print(" attached=");
  Serial.println(leftEyeY.attached() ? "true" : "false");
  Serial.print("Right Y attach result: ");
  Serial.print(rightEyeYAttachResult);
  Serial.print(" attached=");
  Serial.println(rightEyeY.attached() ? "true" : "false");
  Serial.print("X pair configured on D");
  Serial.print(LEFT_EYE_X_PIN);
  Serial.print(" and D");
  Serial.println(RIGHT_EYE_X_PIN);
  Serial.print("Y pair configured on D");
  Serial.print(LEFT_EYE_Y_PIN);
  Serial.print(" and D");
  Serial.println(RIGHT_EYE_Y_PIN);
  Serial.println("All 4 servos set to 90");

  playBeepCount(1);

  connectToWiFi();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  connectToMqtt();
}

void loop() {
  connectToWiFi();
  connectToMqtt();

  mqttClient.loop();

  if (controlModeActive &&
      ((millis() - lastControlEventMs) >= CONTROL_MODE_TIMEOUT_MS)) {
    controlModeActive = false;
    Serial.println("Exited control-mode state (timeout)");
  }

  if (!faceActive && hasSeenFace && !sadQueuedForCurrentAbsence &&
      ((millis() - lastFaceActiveMs) >= FACE_STATE_SOUND_DELAY_MS)) {
    sadSoundPending = true;
    sadQueuedForCurrentAbsence = true;
  }

  if (happySoundPending) {
    happySoundPending = false;
    playHappySound();
  }

  if (sadSoundPending) {
    sadSoundPending = false;
    playSadSound();
  }
}