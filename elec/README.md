# WRO Competition Robot – Complete System Documentation

A full breakdown of the electrical, sensor, and control architecture for the WRO Future Engineers competition robot.

---

## Table of Contents

1. [Main Power System](#1-main-power-system)
2. [Raspberry Pi 5](#2-raspberry-pi-5)
3. [Raspberry Pi ↔ ESP32 UART Communication](#3-raspberry-pi--esp32-uart-communication)
4. [ESP32 Controller](#4-esp32-controller)
5. [N20 Motor and Encoder](#5-n20-motor-and-encoder)
6. [N20 Encoder](#6-n20-encoder)
7. [BN0085 Gyroscope / IMU](#7-bn0085-gyroscope--imu)
8. [PCA9685 Servo Driver](#8-pca9685-servo-driver)
9. [MG996R Steering Servo](#9-mg996r-steering-servo)
10. [I²C Bus](#10-i²c-bus)
11. [LiDAR System](#11-lidar-system)
12. [Ultrasonic Sensor](#12-ultrasonic-sensor)
13. [Ground System](#13-ground-system)
14. [Overall Robot Architecture](#14-overall-robot-architecture)
15. [Control Cycle During a Run](#15-how-everything-works-together-during-a-run)

---

## 1. Main Power System

The robot is powered by an **11.1 V LiPo battery**.

### Power Flow

\`\`\`
11.1 V LiPo Battery
        │
        ▼
   Main Power Switch
        │
        ├──────────────► XY3036 Buck Converter
        │                    │
        │                    └── 5 V / 5 A
        │
        └──────────────► LM2596 Buck Converter
                             │
                             └── 5 V / 2 A
\`\`\`

The key design point in this version of the schematic: **both buck converters receive their input directly from the switched 11.1 V battery rail.** They are parallel power-conversion branches — not one converter feeding the other.

### XY3036

Converts **11.1 V → 5 V / up to 5 A**.

This high-current 5 V supply is used primarily for the **Raspberry Pi 5**, which can draw significantly more current than the smaller electronics.

### LM2596

Independently converts **11.1 V → 5 V / up to 2 A**.

Provides a separate 5 V rail for the lower-power electronics and peripherals.

### Why Two Converters?

Separates the power demands of the Pi from the rest of the electronics:

| Component | Supply |
|---|---|
| Pi 5 | High-current 5 V (XY3036) |
| ESP32 | 5 V input |
| Sensors | 5 V / 3.3 V rails |
| Servo system | 5 V supply |
| Other peripherals | 5 V supply |

All grounds are connected to a common ground reference.

> **Important:** The schematic should clearly show the two converters' inputs as parallel from the 11.1 V rail. Their 5 V outputs should **not** be tied together unless the converters are specifically designed for parallel operation.

---

## 2. Raspberry Pi 5

The Raspberry Pi 5 acts as the robot's high-level computer, handling:

- Camera processing
- Computer vision
- Obstacle detection
- LiDAR processing
- Navigation decisions
- Communication with the ESP32

Power: receives 5 V from the **XY3036** output.

### Camera

The **Pi Camera 5 Wide Angle** connects via the **CSI ribbon cable**, enabling computer-vision tasks such as identifying:

- Red pillars
- Green pillars
- Track/arena features
- Other visual navigation information

---

## 3. Raspberry Pi ↔ ESP32 UART Communication

\`\`\`
Pi Pin 8  TX ─────────► ESP32 RX2
Pi Pin 10 RX ◄───────── ESP32 TX2
Pi GND     ──────────── ESP32 GND
\`\`\`

Two-way communication link:

- **Pi → ESP32:** commands like `SET_SPEED`, `STEER_LEFT`, `STEER_RIGHT`, `STOP`
- **ESP32 → Pi:** encoder speed, motor position, IMU data, status information

**Division of responsibility:**

| Processor | Role |
|---|---|
| Raspberry Pi | High-level intelligence — "Where should the robot go?" |
| ESP32 | Low-level hardware control — "How do I make the motors and steering do that?" |

---

## 4. ESP32 Controller

The **ESP32 DevKitC** is the real-time hardware controller, interfacing with:

- N20 motor
- Encoder
- TB6612FNG
- BN0085 IMU
- PCA9685
- Steering servo

Supplied from the 5 V rail; its onboard regulator steps this down to 3.3 V for the ESP32's logic circuitry.

---

## 5. N20 Motor and Encoder

The robot uses an **N20 600 RPM motor** for propulsion, connected to the **TB6612FNG** motor driver (not directly to the ESP32). The driver supplies motor current while the ESP32 provides control signals.

### TB6612FNG Motor Driver

- **Motor power:** from the main battery rail (the N20 needs more voltage/current than logic electronics provide)
- **Control signals from ESP32:** `AIN1`, `AIN2`, `PWMA` — controlling forward/reverse direction and PWM speed

\`\`\`
ESP32
 │
 ├── AIN1 ──┐
 ├── AIN2 ──┤──► TB6612FNG ───► N20 Motor
 └── PWMA ──┘
\`\`\`

PWM allows the ESP32 to control motor speed.

---

## 6. N20 Encoder

Four encoder connections:

| Pin | Wire Color |
|---|---|
| C1 | Green |
| C2 | Yellow |
| VCC | Black |
| GND | Blue |

The two signal channels (C1/C2) connect to ESP32 GPIOs, providing rotational feedback. Because there are two channels, the ESP32 can determine both **speed** and **direction** of rotation via quadrature decoding — enabling closed-loop control.

Instead of simply commanding "run the motor at 60% PWM," the controller can target "maintain the required wheel speed" — useful for consistent WRO runs.

---

## 7. BN0085 Gyroscope / IMU

The previous **MPU6050** has been replaced by the **BN0085**.

\`\`\`
BN0085
 ├── VCC  → ESP32 3.3 V
 ├── GND  → GND
 ├── SDA  → ESP32 GPIO4
 ├── SCL  → ESP32 I²C clock
 └── PS0  → ESP32 3.3 V
\`\`\`

- **SDA:** BN0085 SDA → ESP32 GPIO4 (GPIO4 is the data line for this interface)
- **VCC:** 3.3 V from the ESP32 3V3 pin
- **PS0:** tied to 3.3 V, configuring the sensor's interface/address mode

### Why the IMU?

Gives the robot an additional orientation source alongside LiDAR. If the robot drifts (e.g., rotates slightly clockwise when it should go straight), the IMU detects the yaw change and the controller compensates via steering or motor control.

---

## 8. PCA9685 Servo Driver

Controls the steering servo via **I²C** from the ESP32.

\`\`\`
ESP32
   │
   │ I²C
   ▼
PCA9685
   │
   │ PWM
   ▼
MG996R Steering Servo
\`\`\`

The PCA9685 generates the PWM signal for the servo, handling it independently from the ESP32's main program loop.

---

## 9. MG996R Steering Servo

Controls the robot's front-wheel steering. Receives power, ground, and a PWM control signal; the ESP32 determines the desired steering angle.

\`\`\`
ESP32 command
     │
     ▼
PCA9685
     │
     ▼
MG996R
     │
     ▼
Front-wheel steering
\`\`\`

---

## 10. I²C Bus

Two primary signals:

- **SDA** – Data
- **SCL** – Clock

Multiple compatible devices can share the bus if addresses and electrical requirements align — in this architecture, this can include the **BN0085** and **PCA9685**.

> **Note:** Since the BN0085's SDA is moved to GPIO4, it should be treated as a separate ESP32 interface if GPIO4 isn't part of the same I²C bus configuration as the PCA9685.

---

## 11. LiDAR System

\`\`\`
LiDAR
   │
  USB
   ▼
USB Translator / USB Adapter
   │
  USB
   ▼
Raspberry Pi 5
\`\`\`

Provides distance measurements around the arena:

\`\`\`
            Front
               0°
               ↑
       -90° ◄ ROBOT ► +90°
        Left          Right
\`\`\`

Enables determination of:

- Distance to walls
- Position relative to boundaries
- Front obstacle distance
- Corner geometry
- Available space
- Pillar locations

Especially useful for the **WRO Future Engineers** navigation system.

---

## 12. Ultrasonic Sensor

**HC-SR04** ultrasonic sensor with four connections: `VCC`, `GND`, `TRIG`, `ECHO`.

| Signal | Pi Pin |
|---|---|
| TRIG | GPIO17 |
| ECHO | GPIO27 |

Provides a complementary distance measurement directly in front of the robot, alongside LiDAR.

---

## 13. Ground System

A shared electrical reference is essential:

\`\`\`
Battery GND
     │
     ├── Buck converters
     ├── ESP32
     ├── TB6612FNG
     ├── PCA9685
     ├── BN0085
     ├── Raspberry Pi
     ├── Ultrasonic sensor
     └── Other electronics
\`\`\`

Without a common ground, signals such as UART, I²C, PWM, and encoder feedback may not have a reliable reference.

---

## 14. Overall Robot Architecture

The robot can be viewed as three layers:

\`\`\`
                ┌──────────────────────────┐
                │      RASPBERRY PI 5      │
                │                          │
                │ Camera                   │
                │ LiDAR                    │
                │ Navigation               │
                │ Computer Vision          │
                │ High-level Decisions     │
                └────────────┬─────────────┘
                             │
                           UART
                             │
                             ▼
                ┌──────────────────────────┐
                │          ESP32           │
                │                          │
                │ Motor Control            │
                │ Encoder Feedback         │
                │ BN0085 IMU               │
                │ Steering Control         │
                └───────┬─────────┬────────┘
                        │         │
                   PWM/Direction  │ I²C
                        │         │
                        ▼         ▼
                   TB6612FNG   PCA9685
                        │         │
                        ▼         ▼
                   N20 Motor   MG996R
\`\`\`

Sensor data flow:

\`\`\`
Camera ──────────► Raspberry Pi
LiDAR ───────────► Raspberry Pi
Ultrasonic ──────► Raspberry Pi
                       │
                       │ UART
                       ▼
                     ESP32
                       ▲
                       │
                  BN0085 IMU
                       │
                  Encoder feedback
\`\`\`

---

## 15. How Everything Works Together During a Run

A typical control cycle:

1. **Sense** — The robot collects data from the camera, LiDAR, ultrasonic sensor, encoder, and BN0085.
2. **Process** —
   - *Raspberry Pi:* camera detections, LiDAR geometry, obstacle position, wall position
   - *ESP32:* encoder measurements, IMU measurements, low-level motor/servo control
3. **Decide** — The Pi determines an action, e.g., "steer slightly left" or "a green pillar is ahead, avoidance maneuver required."
4. **Communicate** — The Pi sends the command to the ESP32 over UART.
5. **Actuate** — The ESP32 changes steering angle via the PCA9685 and adjusts motor PWM via the TB6612FNG.
6. **Feedback** — The encoder and BN0085 provide continuous feedback, allowing the controller to correct the robot's motion in real time.
