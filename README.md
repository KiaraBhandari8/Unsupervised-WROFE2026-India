# Unsupervised-WROFE2026-India

Official repository of Team Unsupervised for the World Robot Olympiad Future Engineers 2026. This project documents the design, development, and implementation of our autonomous vehicle, integrating computer vision, embedded systems, and real-time navigation algorithms to solve the Future Engineers challenge.

## Table of Contents

- [Unsupervised-WROFE2026-India](#unsupervised-wrofe2026-india)
  - [Table of Contents](#table-of-contents)
  - [Team](#team)
  - [About the challenge](#about-the-challenge)
  - [List of Components](#list-of-components)
  - [Robot Pictures](#robot-pictures)
  - [Mobility Management](#mobility-management)
    - [Controlling the Motors](#controlling-the-motors)
  - [Building Instructions](#building-instructions)
  - [Power & Sense Management](#power--sense-management)
    - [Hardware Architecture](#hardware-architecture)
    - [Security Measures](#security-measures)
      - [Current Stabilisation](#current-stabilisation)
  - [Obstacle Management](#obstacle-management)
    - [Vision Methods and Decision Making](#vision-methods-and-decision-making)
    - [1) Image pipeline (inputs used by algorithms)](#1-image-pipeline-inputs-used-by-algorithms)
    - [2) Wall following calculations](#2-wall-following-calculations)
    - [3) Obstacle handling calculations](#3-obstacle-handling-calculations)
    - [4) Crash detection](#4-crash-detection)
    - [5) Arbitration: choosing the action](#5-arbitration-choosing-the-action)
    - [6) Tuning notes](#6-tuning-notes)
    - [Block diagrams](#block-diagrams)
    - [Extras](#extras)
  - [Software Key Components](#software-key-components)
  - [Possible Improvements](#possible-improvements)

## Team

| Name           | Profile                                | Role        |
| -------------- | -------------------------------------- | ----------- |
| Kiara Bhandari | Grade 10 @ Oberoi International School | Team Member |
| Shubh Gupta    | Grade 10 @ Chatrabhuj Narsee School    | Team Member |
| Dhanak Seth    | Grade 8 @ Vibgyor High School          | Team Member |
| Vinay Ummadi   | Mentor @ MakerWorks Lab                | Team Mentor |

## About the Challenge

The World Robot Olympiad (WRO) is an international robotics competition that encourages students to develop problem-solving, programming, and engineering skills. The Future Engineers category is designed for students aged 14–22 years and focuses on autonomous driving. The challenge simulates real-world traffic conditions and requires teams to design and program a fully autonomous robot car.

The challenge requires students to construct an autonomous robot which will undergo 2 rounds. First, an open round challenge where the robot would need to complete 3 rounds around the arena within the time limit of 3 minutes (180 seconds). The second round, which is the obstacle round consists of navigating through red and green pillars, where the robot would move from the left of the green pillar and from right of the red pillar. The robot should once again, not exceed a time limit of 3 minutes.

## List of Components

| Name of Component | Quantity |
| ---- | ---- |
| Raspberry Pi 5 (Cooling Fan + SD Card) | 1 |
| Raspberry Pi Camera Module 3 Wide | 1 |
| ESP32 Development Board | 1 |
| YDLidar T-Mini Plus LiDAR | 1 |
| PCA9685 16-Channel PWM Servo Driver | 1 |
| TB6612FNG Dual Motor Driver | 1 |
| GY-87 10-DOF Multi-Sensor IMU Module | 1 |
| Silicon Labs CP2102 USB-to-UART Bridge | 1 |
| MG996 Servo Motor | 1 |
| N20 200 rpm Gear Motors | 2 |
| LiPo 3s 11.1v 2200 mAh Battery | 1 |
| LM2596 Step-Down (Buck) DC-DC Switching Voltage Regulator Integrated Circuit (ESP32) | 1 |
| XY-3606 DC-DC Step-Down Buck Converter Module (Raspberry Pi) | 1 |
| RC Car Rear Differential | 1 |
| N20 wheels | 4 |
| Lazy Susan Turntable Bearings | 1 |

## Robot Pictures

## Mobility Management

The robot uses a rear-wheel-drive system consisting of two N20 200 RPM gear motors connected to an RC car rear differential. The differential transfers the motors' motion to the rear wheels while allowing the wheels to rotate at different speeds during turns. Steering is provided by an MG996 servo motor connected to the front steering mechanism. The robot uses four N20 wheels, with the rear wheels being driven and the front wheels used for steering. A Lazy Susan turntable bearing supports the steering assembly.

### Controlling the Motors

The two N20 gear motors are controlled by the ESP32 through the TB6612FNG dual motor driver. The Raspberry Pi sends movement commands to the ESP32, which controls the motors according to the required speed and direction. The MG996 steering servo is controlled by the ESP32 through the PCA9685 PWM servo driver.

## Building Instructions

## Power & Sense Management

The robot is powered by a LiPo 3S 11.1 V 2200 mAh battery. DC-DC buck converters regulate the battery voltage to suitable levels for the Raspberry Pi 5 and ESP32. The regulated power supplies allow the computing, sensing, and control components to operate reliably.

### Hardware Architecture

The Raspberry Pi 5 acts as the main computing unit. It processes data from the Raspberry Pi Camera Module 3 Wide and YDLidar T-Mini Plus LiDAR and runs the robot's navigation algorithms.

The ESP32 handles real-time control of the robot. It receives movement commands from the Raspberry Pi and controls the N20 gear motors through the TB6612FNG motor driver. Steering is controlled by the MG996 servo through the PCA9685 PWM servo driver.

The GY-87 10-DOF IMU provides motion and orientation data to assist with navigation and movement monitoring.

Communication between the Raspberry Pi and ESP32 allows high-level navigation decisions made by the Raspberry Pi to be translated into real-time motor and steering commands by the ESP32.

### Security Measures

#### Current Stabilisation

The robot uses DC-DC buck converters to regulate the voltage supplied to its electronic components. This prevents the higher battery voltage from being supplied directly to components that require lower operating voltages.

The regulated power system helps maintain stable operation of the Raspberry Pi, ESP32, sensors, and control electronics during operation.

## Obstacle Management

The robot uses a combination of computer vision, LiDAR, and IMU data to detect obstacles and determine its path through the arena.

### Vision Methods and Decision Making

The Raspberry Pi Camera Module 3 Wide is used to identify the coloured obstacles in the arena. Image processing is used to detect red and green obstacles and determine their position relative to the robot.

The YDLidar T-Mini Plus provides distance measurements around the robot. These measurements are used for wall following, obstacle detection, and determining the available space around the robot.

The GY-87 IMU provides motion and orientation data, which can be used to detect changes in the robot's movement and assist with navigation.

The navigation system combines information from these sensors to determine the appropriate steering angle and motor speed. The Raspberry Pi processes the sensor data and sends movement commands to the ESP32, which controls the steering servo and drive motors.

### 1) Image Pipeline (Inputs Used by Algorithms)

The camera captures images of the arena, which are processed on the Raspberry Pi.

The image-processing pipeline consists of:
1. Capturing an image from the camera.
2. Converting the image into a suitable colour space.
3. Detecting the relevant coloured regions.
4. Identifying red and green obstacles.
5. Determining the position of detected obstacles.
6. Passing the resulting information to the navigation algorithm.

### 2) Wall Following Calculations

LiDAR measurements are used to determine the robot's distance from the walls.

The navigation algorithm compares the measured distance with the desired wall distance and calculates a steering correction. This allows the robot to maintain a suitable position within the lane while moving around the arena.

### 3) Obstacle Handling Calculations

When an obstacle is detected, the robot determines its colour and position using the camera.

The navigation system then selects the appropriate direction around the obstacle while considering LiDAR distance measurements and the robot's current movement.

### 4) Crash Detection

The robot uses sensor data to monitor unexpected changes in movement. IMU measurements can be used to identify sudden changes in acceleration or orientation that may indicate a collision or abnormal movement.

### 5) Arbitration: Choosing the Action

The navigation system combines information from the camera, LiDAR, and IMU to determine the robot's next action.

The resulting command contains the required steering angle and motor speed. The Raspberry Pi sends this command to the ESP32, which controls the steering servo and motors.

### 6) Tuning Notes

The steering and navigation parameters are tuned through repeated testing on the arena. Parameters such as steering corrections, motor speed, wall-following distance, and obstacle detection thresholds are adjusted to improve stability and reduce unnecessary corrections.

