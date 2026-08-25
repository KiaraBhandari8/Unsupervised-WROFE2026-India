# Mechanical Design Approach 

## 1. Basic Mechanical Choices

##### 1.1 Wheel Choice 
We chose the 43mm N20-compatible rubber wheels because they provide a good balance between grip, stability, manoeuvrability and size. The rubber tyre gives the robot good traction on the surface, reducing slipping and allowing more consistent movement and turning. The 43mm diameter is large enough to provide adequate ground clearance and smooth movement, while still being compact enough to fit within our robot’s design without taking up excessive space. Their 18mm width provides sufficient contact with the surface for stability without adding unnecessary weight. At only 18g per wheel, they help keep the robot lightweight, while their 3kg loading capacity provides more than enough support for our robot. The wheels are also directly compatible with N20 gear motors and have a 3mm shaft hole, simplifying the mechanical assembly. Additionally, the 12-pulse-per-revolution encoder provides accurate wheel rotation feedback, which can help with precise movement and distance control.

<img width="460" height="400" alt="WhatsApp Image 2026-08-22 at 11 32 25" src="https://github.com/user-attachments/assets/8d3e13d1-a5d2-414a-b0a1-220d54d10643" />

##### 1.2 Steering System 
- Prototype:
- Servo: 

##### 1.3 Differential Gear (Rear Wheels)
We chose an RC car rear differential gear to efficiently transfer power from the motor to both rear wheels while allowing the wheels to rotate at different speeds during turns. This is important because the outer wheel travels a greater distance than the inner wheel when the robot turns. The differential therefore reduces wheel slipping and friction, allowing smoother and more controlled turns.

<img width="1668" height="2157" alt="differential gear" src="https://github.com/user-attachments/assets/9540fc8b-eaa4-4592-8ffc-74a4ef8c27dd" />

In our design, we changed the orientation of the differential gear to improve its efficiency and integration with the rest of the drivetrain. We also increased the distance between the LiDAR stand and the differential gear compared to last year’s design. Previously, the smaller spacing caused the differential assembly to come too close to the LiDAR stand, creating a risk of interference or collision. Increasing this clearance gives the LiDAR more space and prevents mechanical components from obstructing its operation.

This modification also makes the overall drivetrain more reliable by reducing unnecessary contact between components and providing better separation between the sensing and drive systems. The differential gear therefore contributes to both the robot’s turning performance and the improved mechanical layout of the final design.

##### 1.4 Dimension Choices 
**Dimensions:** **50 cm × 29.5 cm × 22 cm**  
**Length × Width × Height**

**Why We Chose These Dimensions:**

We chose the dimensions of **50 cm × 29.5 cm × 22 cm** to provide a balance between **stability, manoeuvrability, and component placement**. The 50 cm length provides enough space to accommodate the drivetrain, battery, electronics, and sensors while maintaining a compact overall design. The 29.5 cm width provides sufficient stability during movement and turning without making the robot unnecessarily wide. The 22 cm height keeps the robot's centre of mass relatively low while providing enough clearance for mounting the camera, LiDAR, and other electronic components. These dimensions also allow the robot to remain compact enough for efficient navigation around the track.

