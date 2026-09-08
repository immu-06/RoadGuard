# RoadGuard

RoadGuard is an intelligent road accident detection and emergency alert system.

## Project Overview

RoadGuard detects possible vehicle crashes using an ESP32 and MPU6050 sensor.

When a crash is detected:

1. MPU6050 captures sudden acceleration/rotation.
2. ESP32 processes the sensor data.
3. NEO-6M GPS provides the location.
4. Crash information is transmitted wirelessly using ESP-NOW.
5. The receiving ESP32 displays the crash information.
6. The Flutter application can receive/store crash information.
7. Supabase is used as the backend.
8. Emergency contacts can be notified through the backend.

## Architecture

```text
MPU6050
   |
   v
ESP32 Node A
   |
   | Crash Detection
   | GPS Location
   |
   | ESP-NOW
   v
ESP32 Node B
   |
   v
OLED Display
   |
   v
Flutter Application
   |
   v
Supabase Backend
   |
   v
Emergency Notification
