# Underground-Tunnel-Explorer-Robot

This project presents a compact autonomous robot designed to explore narrow, confined, and potentially hazardous underground passages.

The robot combines obstacle avoidance, tilt safety, target tracking, and outdoor GPS monitoring through a dual-controller architecture using Arduino and ESP32.

---

## Problem Description

Underground passages such as those discovered in Jeddah Al-Balad can be difficult and unsafe for direct human inspection.

These environments may contain:

- Narrow pathways
- Unknown layouts
- Uneven surfaces
- Obstructions
- Limited accessibility

The project addresses the need for a small robotic platform that can move through tunnel-like environments, react to obstacles, maintain safe orientation, and support remote location monitoring.

---

## Proposed System

The robot uses two controllers:

- **Arduino** for movement, sensor fusion, obstacle avoidance, tilt safety, and HuskyLens-based target tracking.
- **ESP32** for GPS decoding, Wi-Fi communication, and displaying outdoor coordinates through a local web page.

The main navigation behavior is reactive and rule-based. The robot continuously reads its sensors and selects its next movement based on the current environment.

---

## System Architecture

```text
Ultrasonic Sensor ─┐
IR Distance Sensor ├──> Arduino ───> L298N Motor Driver ───> DC Motors
MPU6050 IMU ───────┤
HuskyLens Camera ──┘

GPS Module ───> ESP32 ───> Wi-Fi Access Point ───> Web Interface
```

---

## Main Components

| Component | Purpose |
|---|---|
| Arduino | Main controller for navigation, safety logic, and motor commands |
| ESP32 | GPS processing, Wi-Fi access point, and web server |
| Ultrasonic Sensor | Measures obstacle distance and scans right, front, and left |
| Servo Motor | Rotates the ultrasonic sensor for multi-direction scanning |
| IR Distance Sensor | Provides an additional front-distance reading |
| MPU6050 IMU | Detects unsafe tilt using the accelerometer Z-axis |
| HuskyLens | Detects and follows a trained visual target |
| GPS Module | Provides outdoor coordinates |
| L298N Motor Driver | Controls the left and right motor groups |
| DC Motors | Move and steer the robot |

---

## Navigation Logic

The robot follows a simple reactive sequence:

```text
Check IMU Safety
      ↓
Read Front Distance
      ↓
Path Clear?
 ┌────┴────┐
Yes       No
 ↓         ↓
Move    Stop and Scan
Forward  Right and Left
              ↓
      Choose the Most Open Side
              ↓
             Turn
```

The robot moves forward when the path is clear and stops when an obstacle is detected within **15 cm**.

When blocked, the servo rotates the ultrasonic sensor to scan both sides. The robot compares the measured distances and turns toward the direction with more free space.

---

## Sensor Fusion

The project uses rule-based sensor fusion rather than machine learning.

### Distance Fusion

The ultrasonic and IR sensors provide front-distance information. Their readings are combined to improve obstacle detection and reduce the effect of invalid measurements.

### Direction Selection

The servo-mounted ultrasonic sensor measures:

- Right distance
- Front distance
- Left distance

The robot selects the direction with the largest open space.

### Tilt Safety

The MPU6050 monitors the accelerometer Z-axis.

If the reading drops below the configured safety threshold, the motors stop immediately to reduce the risk of tipping or collision.

### Visual Target Tracking

When HuskyLens detects the trained target with ID `1`, the robot adjusts its motion according to the target's horizontal position:

- Target on the left → turn left
- Target on the right → turn right
- Target near the center → move forward

Obstacle avoidance remains active during target tracking.

---

## GPS and Wi-Fi Tracking

The ESP32 reads GPS data through `Serial2` using TinyGPS++.

It creates a local Wi-Fi access point named:

```text
Robot-GPS
```

A nearby device can connect to the robot and open a local webpage showing:

- Latitude
- Longitude
- Number of satellites
- A direct Google Maps link

GPS is intended for outdoor tracking and does not control indoor tunnel navigation.

---

## Path Planning

The main physical navigation method is reactive and does not require a predefined map, SLAM, or AI-based planning.

The code also includes an optional **A\*** assumption mode for testing movement on a predefined 5 × 5 grid. This mode calculates a path from a start cell to a goal cell and converts the path into robot movement commands.

---

## Control Paradigm

The robot follows a hybrid control design:

- **Reactive layer:** obstacle avoidance, distance scanning, and immediate tilt safety.
- **Goal-oriented layer:** HuskyLens target following and GPS-based outdoor monitoring.

This structure allows the robot to respond quickly to nearby hazards while supporting higher-level tracking functions.

---

## Validation

The core movement and obstacle-avoidance logic was tested in TinkerCAD before hardware integration.

The simulation validated:

- Forward movement
- Obstacle detection
- Servo-based right and left scanning
- Selection of the safer direction
- Motor-control logic

Some components, including HuskyLens, GPS, ESP32, and MPU6050, required physical implementation because they are not fully supported in TinkerCAD.

---

## Project Files

| File | Description |
|---|---|
| `Robot_SensorFusion_Husky_IMU.ino` | Main Arduino code for sensor fusion, motor control, tilt safety, target tracking, reactive navigation, and optional A* mode |
| `Robot_GPS_WebServer.ino` | ESP32 code for GPS decoding, Wi-Fi access point creation, and browser-based coordinate display |
| `A_Algorithm.ino` | Arduino navigation version containing the predefined-grid A* implementation and integrated robot control logic |

---

## Main Features

- Autonomous obstacle avoidance
- Multi-direction distance scanning
- Ultrasonic and IR distance fusion
- IMU-based tilt protection
- HuskyLens target following
- Outdoor GPS tracking
- Local Wi-Fi web interface
- Rule-based real-time navigation
- Optional A* grid-planning mode
- TinkerCAD validation of core movement logic

---

## Conclusion

The project demonstrates how multiple sensors and controllers can be combined to create a compact robot for confined-environment exploration.

The Arduino manages immediate movement and safety decisions, while the ESP32 provides outdoor GPS monitoring through a wireless interface. The result is a modular robotic platform capable of obstacle avoidance, tilt protection, target tracking, and real-time location reporting.
