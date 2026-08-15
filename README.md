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
