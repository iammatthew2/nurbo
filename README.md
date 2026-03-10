# Nurbo

An Arduino Nano IoT 33 project that drives a two-eye mechanism using four servos, controlled over MQTT. Similar to [Puddles](../puddles2), but with two independently articulated eyes.

## Hardware

- **Board:** Arduino Nano 33 IoT
- **Left Eye Servo X (horizontal):** SG90 micro servo
- **Left Eye Servo Y (vertical):** SG90 micro servo
- **Right Eye Servo X (horizontal):** SG90 micro servo
- **Right Eye Servo Y (vertical):** SG90 micro servo
- **Piezo buzzer:** for sound feedback

## How It Works

Nurbo connects to Wi-Fi and subscribes to MQTT topics:

- **Tracking topic** — receives face-tracking coordinates from a camera system and maps them to servo angles so both eyes follow a detected face.
- **Control topic** — accepts manual commands to position servos directly, plus key commands for test sweeps.

Both eyes can move in sync for tracking, or be controlled independently.

Built with [PlatformIO](https://platformio.org/).

## Libraries

- Servo
- WiFiNINA
- PubSubClient (MQTT)
- ArduinoJson