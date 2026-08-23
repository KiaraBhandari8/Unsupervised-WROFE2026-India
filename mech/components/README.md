### Raspberry Pi 5

**Raspberry Pi 5 (Cooling Fan + SD Card):**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="66.56" alt="Raspberry Pi 5" src="https://github.com/user-attachments/assets/e4878e2a-ee42-4cbb-9076-6eee5a91137b" /> | 1. **Name:** Raspberry Pi 5<br><br>2. **Processor:** Quad-core 64-bit Arm Cortex-A76<br><br>3. **Storage:** MicroSD Card<br><br>4. **Cooling:** Active cooling fan<br><br>5. **Use:** Main processing and navigation |

The **Raspberry Pi 5** acts as the brain of the robot. It processes the camera and LiDAR data, runs the computer vision and navigation algorithms, makes movement decisions, and sends the resulting commands to the **ESP32**. The ESP32 then handles the low-level motor and actuator control.

[Link](https://robu.in/product/raspberry-pi-5-model-8gb/)

---

### Raspberry Pi Camera Module 3 Wide

**Raspberry Pi Camera Module 3 Wide:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="62" height="100" alt="Raspberry Pi Camera Module 3 Wide" src="https://github.com/user-attachments/assets/e67ea9f3-6510-4a26-8285-e8302a622a39" /> | 1. **Name:** Raspberry Pi Camera Module 3 Wide<br><br>2. **Sensor:** Sony IMX708<br><br>3. **Field of View:** Wide-angle<br><br>4. **Interface:** CSI<br><br>5. **Use:** Image capture and computer vision |

The **Raspberry Pi Camera Module 3 Wide** provides visual input to the robot. Its wide field of view allows the robot to capture a larger portion of the track, which is useful for **track detection, object identification, and navigation**.

[Link](https://robu.in/product/raspberry-pi-camera-module-3-wide/)

---

### ESP32 Development Board

**ESP32 Development Board:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="ESP32 Development Board" src="https://github.com/user-attachments/assets/d5deb83d-41c4-404e-a340-20d3ba21cfaf" /> | 1. **Name:** ESP32 Development Board<br><br>2. **Type:** Microcontroller<br><br>3. **Communication:** Serial communication with Raspberry Pi 5<br><br>4. **Use:** Motor and actuator control |

The **ESP32** acts as the robot's low-level motor controller. It receives movement commands from the **Raspberry Pi 5**, which performs the main vision, sensor processing, decision-making, and navigation. The ESP32 converts these commands into control signals for the **motor driver and steering servo**, allowing the robot to physically execute the decisions made by the Raspberry Pi.

[Link](https://robu.in/product/esp32-s3-devkit-esp32-s3-wroom-1-n16r8/)

---

### YDLidar T-Mini Plus LiDAR

**YDLidar T-Mini Plus LiDAR:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="YDLidar T-Mini Plus LiDAR" src="https://github.com/user-attachments/assets/0a3fde64-41e3-4732-825b-1a5cbe024fa0" /> | 1. **Name:** YDLidar T-Mini Plus<br><br>2. **Type:** 2D LiDAR<br><br>3. **Scan Frequency:** Up to 10 Hz<br><br>4. **Interface:** USB / Serial<br><br>5. **Use:** Distance measurement and obstacle detection |

The **YDLidar T-Mini Plus** provides 2D distance measurements around the robot. The LiDAR data is used for **obstacle detection, wall following, collision avoidance, and navigation decisions**.

[Link](https://robu.in/product/ydlidar-t-mini-plus-lidar-sensor/)

---

### PCA9685 16-Channel PWM Servo Driver

**PCA9685 16-Channel PWM Servo Driver:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="PCA9685 16-Channel PWM Servo Driver" src="https://github.com/user-attachments/assets/f667c49a-ed28-44c9-a9d9-43d50fd700eb" /> | 1. **Name:** PCA9685 16-Channel PWM Servo Driver<br><br>2. **Channels:** 16 PWM channels<br><br>3. **Interface:** I²C<br><br>4. **I²C Address:** 0x40<br><br>5. **Use:** Servo control |

The **PCA9685** is used to generate PWM signals for controlling the robot's servo motor. It communicates with the controller through the **I²C interface**, providing precise control of the steering mechanism.

[Link](https://robu.in/product/16-channel-12-bit-pwmservo-driver-i2c-interface-pca9685-arduino-raspberry-pi/)

---

### TB6612FNG Dual Motor Driver

**TB6612FNG Dual Motor Driver:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="TB6612FNG Dual Motor Driver" src="https://github.com/user-attachments/assets/4f52e1c2-4c8d-45e9-8cb3-94717e99e67c" /> | 1. **Name:** TB6612FNG Dual Motor Driver<br><br>2. **Type:** Dual H-Bridge motor driver<br><br>3. **Channels:** 2<br><br>4. **Control:** PWM speed control<br><br>5. **Use:** DC motor control |

The **TB6612FNG** is used to control the robot's DC motors. It allows the ESP32 to control the **direction and speed** of the motors using PWM signals.

[Link](https://robu.in/product/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/)

---

### GY-87 10-DOF Multi-Sensor IMU Module

**GY-87 10-DOF Multi-Sensor IMU Module:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="GY-87 IMU Module" src="https://github.com/user-attachments/assets/06444ccd-9718-44a6-9ecc-b5ebb76ae830" /> | 1. **Name:** GY-87 10-DOF IMU Module<br><br>2. **Sensors:** Accelerometer, gyroscope, magnetometer and barometer<br><br>3. **Interface:** I²C<br><br>4. **Degrees of Freedom:** 10-DOF<br><br>5. **Use:** Motion and orientation sensing |

The **GY-87 IMU** provides information about the robot's movement and orientation. It can be used to monitor **rotation, acceleration, and changes in orientation** during navigation.

[Link](https://robu.in/product/mpu6050hmc5883lbmp180-10dof-3-axis-gyro-3-axis-acceleration-3-axis-magnetic-field-air-pres/)

---

### Silicon Labs CP2102 USB-to-UART Bridge

**Silicon Labs CP2102 USB-to-UART Bridge:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="CP2102 USB-to-UART Bridge" src="https://github.com/user-attachments/assets/51b86460-cb8d-4ee2-b4d7-cae04b5f1223" /> | 1. **Name:** CP2102 USB-to-UART Bridge<br><br>2. **Type:** USB-to-UART converter<br><br>3. **Interface:** USB / UART<br><br>4. **Use:** Serial communication<br><br>5. **Application:** Programming and debugging |

The **CP2102** is used as a USB-to-UART interface for communication with the microcontroller and for **programming, debugging, and serial communication**.

[Link](https://robu.in/product/cp2102-gmr-skyworks-silicon-labs-12mbps-transceiver-wqfn-28-ep5x5-usb-converters-rohs/)

---

### HC-SR04 Ultrasonic Sensor

**HC-SR04 Ultrasonic Sensor:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="HC-SR04 Ultrasonic Sensor" src="https://github.com/user-attachments/assets/6b3e06b6-78e6-4789-aa50-27cb1e9129cb" /> | 1. **Name:** HC-SR04 Ultrasonic Sensor<br><br>2. **Type:** Ultrasonic distance sensor<br><br>3. **Operating Voltage:** 5 V<br><br>4. **Interface:** Trigger and Echo<br><br>5. **Use:** Distance measurement |

The **HC-SR04** measures distance using ultrasonic waves. It can provide an additional proximity measurement for detecting objects near the robot.

[Link](https://robu.in/product/3-3-5-5v-hc-sr04-ultrasonic-sensor-4pin/)

---

### MG996 Servo Motor

**MG996 Servo Motor:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="MG996 Servo Motor" src="https://github.com/user-attachments/assets/4e8ed4f9-2cff-482b-ac7c-e2f42677f6e7" /> | 1. **Name:** MG996 Servo Motor<br><br>2. **Type:** Digital servo motor<br><br>3. **Control:** PWM<br><br>4. **Rotation:** Approximately 180°<br><br>5. **Use:** Steering control |

The **MG996 Servo Motor** is used to control the robot's steering mechanism. Its position is controlled using PWM signals generated through the **PCA9685 servo driver**.

[Link](https://robu.in/product/towerpro-mg995-servo-high-speed-digital-metal-gear-servo-motor-cnc-aluminum-steering-servo-horn-arm-good-quality/)

---

### N20 200 RPM Gear Motors

**N20 200 RPM Gear Motors:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="N20 Gear Motors" src="https://github.com/user-attachments/assets/79ed59ec-01e8-4c4c-be57-d74fc5afe898" /> | 1. **Name:** N20 DC Gear Motor<br><br>2. **Quantity:** 2<br><br>3. **Speed:** Approximately 200 RPM<br><br>4. **Type:** DC geared motor<br><br>5. **Use:** Propulsion |

Following an evaluation of different motors, we selected the **N20 200 RPM Gear Motors** for propulsion. Their compact size and geared design provide suitable torque while keeping the robot lightweight. The motors are connected to the **TB6612FNG motor driver** for speed and direction control.

[Link](https://robu.in/product/n20-6v-100-rpm-micro-metal-gear-box-dc-motor-2/)

---

### LiPo 3S 11.1V 2200mAh Battery

**LiPo 3S 11.1V 2200mAh Battery:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="LiPo 3S 11.1V 2200mAh Battery" src="https://github.com/user-attachments/assets/975ba1bc-026f-4a03-b727-340d5b2d7c12" /> | 1. **Name:** LiPo 3S Battery<br><br>2. **Nominal Voltage:** 11.1 V<br><br>3. **Capacity:** 2200 mAh<br><br>4. **Cell Count:** 3S<br><br>5. **Use:** Main power source |

The **3S LiPo battery** serves as the primary power source for the robot. Its relatively high energy density allows it to power the motors and electronic systems while maintaining a compact form factor.

[Link](https://robu.in/product/orange-2200mah-3s-30c60c-lithium-polymer-battery-pack-lipo/)

---

### LM2596 Step-Down Buck Converter

**LM2596 Step-Down Buck Converter:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="LM2596 Buck Converter" src="https://github.com/user-attachments/assets/3a850fdf-1e0b-4753-a041-a333159d0240" /> | 1. **Name:** LM2596 Step-Down Converter<br><br>2. **Type:** DC-DC buck converter<br><br>3. **Input:** Higher DC voltage<br><br>4. **Output:** Adjustable lower DC voltage<br><br>5. **Use:** ESP32 power regulation |

The **LM2596 buck converter** reduces the battery voltage to a suitable regulated voltage for the **ESP32 and associated electronics**. This provides a more appropriate supply voltage while improving power efficiency.

[Link](https://robu.in/product/lm2596-buck-step-power-converter-module-dc-4-040-1-3-37v-led-voltmeter/)

---

### XY-3606 DC-DC Step-Down Buck Converter

**XY-3606 DC-DC Step-Down Buck Converter:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="XY-3606 Buck Converter" src="https://github.com/user-attachments/assets/8af8d885-89c7-4b68-9acc-c355d38f7582" /> | 1. **Name:** XY-3606 DC-DC Buck Converter<br><br>2. **Type:** Step-down voltage regulator<br><br>3. **Input:** Higher DC voltage<br><br>4. **Output:** Adjustable lower DC voltage<br><br>5. **Use:** Raspberry Pi power regulation |

The **XY-3606 buck converter** is used to regulate the battery voltage to a suitable level for powering the **Raspberry Pi 5**. This allows the computing system to receive a stable power supply from the main battery.

[Link](https://robu.in/product/24v-12v-to-5v-5a-power-module-dc-dc-xy-3606-power-converter/)

---

### RC Car Rear Differential

**RC Car Rear Differential:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="RC Car Rear Differential" src="https://github.com/user-attachments/assets/783b6d71-c0a7-4619-8c8f-c254a46ddcd5" /> | 1. **Name:** RC Car Rear Differential<br><br>2. **Type:** Mechanical differential<br><br>3. **Use:** Power distribution to rear wheels<br><br>4. **Application:** Rear-wheel drive system |

The **RC Car Rear Differential** transfers power from the motors to the rear wheels while allowing the wheels to rotate at different speeds when turning. This improves the robot's ability to negotiate corners smoothly.

[Link](https://www.daddydrones.in/mjx-hypergo-16420y-metal-differential-1-14-rc-cars-14301-14303?srsltid=AfmBOoroIT2cUJfjCjysMz7nRBXE7Pnv0Dtjo55qHZ8IfWWZk6aML6wZ)

---

### N20 Wheels

**N20 Wheels:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="N20 Wheels" src="https://github.com/user-attachments/assets/e54c118c-2956-4393-8d43-4b2afd229bf5" /> | 1. **Name:** N20 Wheels<br><br>2. **Quantity:** 4<br><br>3. **Type:** Robot/RC wheels<br><br>4. **Compatibility:** N20 gear motors<br><br>5. **Use:** Ground contact and propulsion |

The **N20 wheels** provide the robot with traction and allow the motor torque to be transferred to the track surface. Four wheels are used to provide stable contact with the ground.

[Link](https://robu.in/product/3pi-miniq-car-wheel-tyre-42mm-n20-dc-gear-motor-wheel/)

---

### Lazy Susan Turntable Bearings

**Lazy Susan Turntable Bearings:**

| **Component Image** | **Specifications** |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img width="100" height="100" alt="Lazy Susan Turntable Bearing" src="https://github.com/user-attachments/assets/236d9dd5-82f3-4874-a29f-3ce52fc81cbf" /> | 1. **Name:** Lazy Susan Turntable Bearing<br><br>2. **Type:** Rotational bearing<br><br>3. **Use:** Low-friction rotation<br><br>4. **Application:** Mechanical support<br><br>5. **Function:** Allows smooth rotational movement |

The **Lazy Susan turntable bearing** is used as a mechanical support component to allow smooth rotational movement while reducing friction. It contributes to the mechanical stability and movement of the robot.

[Link](https://www.amazon.in/VXB-Capacity-Bearing-Turntable-Bearings/dp/B002TIKEQ6)

