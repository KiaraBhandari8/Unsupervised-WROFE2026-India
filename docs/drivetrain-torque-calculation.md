# Drivetrain Torque and Driving Force Calculation

## 1. Objective

The purpose of this calculation is to determine the **drivetrain torque and driving force** of the WRO robot and to verify whether the two N20 gear motors provide sufficient torque for the robot's mass and wheel size.

The robot uses:

- 2 × N20 200 RPM gear motors
- 42 mm diameter wheels
- RC car rear differential
- 1.3 kg total robot mass
- 11.1 V, 2200 mAh 3S LiPo battery

The fundamental relationship between torque and tangential force is:

$$
\tau = F \times r
$$

where:

- $\tau$ = torque in N·m
- $F$ = tangential force in N
- $r$ = wheel radius in m

---

## 2. Given Parameters

| Parameter | Symbol | Value |
|---|---:|---:|
| Number of motors | $n$ | 2 |
| Motor speed | $N$ | 200 RPM |
| Rated torque per motor | $\tau_m$ | 0.52 kg·cm |
| Robot mass | $m$ | 1.3 kg |
| Wheel diameter | $D$ | 42 mm |
| Wheel radius | $r$ | 21 mm |
| Estimated drivetrain efficiency | $\eta$ | 85% |
| Gravitational acceleration | $g$ | 9.81 m/s² |


---

# 3. Calculate the Wheel Radius

The wheel diameter is:

$$
D = 42\text{ mm}
$$

The radius is half of the diameter:

$$
r = \frac{D}{2}
$$

$$
r = \frac{42}{2} = 21\text{ mm}
$$

Convert millimetres to metres:

$$
r = \frac{21}{1000}
$$

Therefore:

$$
\boxed{r = 0.021\text{ m}}
$$

The wheel radius used for the torque calculations is therefore:

$$
\boxed{0.021\text{ m}}
$$

---

# 4. Convert Motor Torque to N·m

The rated torque of one N20 motor is assumed to be:

$$
\tau_m = 0.52\text{ kg·cm}
$$

The conversion factor is:

$$
1\text{ kg·cm} = 0.0980665\text{ N·m}
$$

Therefore:

$$
\tau_m = 0.52 \times 0.0980665
$$

$$
\tau_m = 0.05099\text{ N·m}
$$

Therefore, the rated torque of **one motor** is:

$$
\boxed{\tau_m \approx 0.051\text{ N·m}}
$$

---

# 5. Calculate Total Motor Torque

The robot uses two N20 motors.

Assuming both motors contribute equally to the drivetrain:

$$
\tau_{total} = \tau_1 + \tau_2
$$

Since:

$$
\tau_1 = \tau_2 = 0.051\text{ N·m}
$$

we obtain:

$$
\tau_{total} = 0.051 + 0.051
$$

$$
\boxed{\tau_{total} = 0.102\text{ N·m}}
$$

Thus, the combined rated torque supplied by the two motors is:

$$
\boxed{0.102\text{ N·m}}
$$

before mechanical losses.

---

# 6. Account for Drivetrain Efficiency

The motors do not transfer 100% of their torque to the wheels.

Torque can be lost through:

- Motor gearbox
- Rear differential
- Bearings
- Axle
- Gear mesh
- Wheel connections

Assuming an overall mechanical efficiency of:

$$
\eta = 85\% = 0.85
$$

the effective wheel torque is:

$$
\tau_{wheel} = \tau_{total} \times \eta
$$

Substituting:

$$
\tau_{wheel} = 0.102 \times 0.85
$$

$$
\boxed{\tau_{wheel} = 0.0867\text{ N·m}}
$$

Therefore, the estimated torque actually available at the wheels is:

$$
\boxed{\tau_{wheel} \approx 0.087\text{ N·m}}
$$

---

# 7. Calculate the Driving Force

The torque at the wheels produces a tangential force at the contact point between the wheels and the ground.

Starting with:

$$
\tau = F \times r
$$

Rearranging:

$$
F = \frac{\tau}{r}
$$

Using:

$$
\tau = 0.0867\text{ N·m}
$$

and:

$$
r = 0.021\text{ m}
$$

we get:

$$
F = \frac{0.0867}{0.021}
$$

Therefore:

$$
\boxed{F \approx 4.13\text{ N}}
$$

The estimated total driving force is:

$$
\boxed{F_{drive} \approx 4.13\text{ N}}
$$

---

# 8. Calculate the Robot's Weight

The robot mass is:

$$
m = 1.3\text{ kg}
$$

Weight is calculated using:

$$
W = mg
$$

Therefore:

$$
W = 1.3 \times 9.81
$$

$$
\boxed{W = 12.75\text{ N}}
$$

The robot therefore exerts approximately **12.75 N** of downward force due to gravity.

---

# 9. Estimate Theoretical Acceleration

Using Newton's second law:

$$
F = ma
$$

Rearranging:

$$
a = \frac{F}{m}
$$

Using the estimated driving force:

$$
F = 4.13\text{ N}
$$

and:

$$
m = 1.3\text{ kg}
$$

we obtain:

$$
a = \frac{4.13}{1.3}
$$

$$
\boxed{a \approx 3.18\text{ m/s}^2}
$$

This is the **idealized motor-limited acceleration**.

In real operation, the acceleration will be lower because:

- Motor torque decreases as motor speed increases
- Tyre traction limits may occur
- Battery voltage drops under load
- Rolling resistance is present
- Mechanical losses vary with load
- The motor torque specification may represent stall or rated conditions rather than the torque available at 200 RPM

---

# 10. Calculate Theoretical Maximum Speed

The wheel diameter is:

$$
D = 0.042\text{ m}
$$

The circumference of the wheel is:

$$
C = \pi D
$$

Therefore:

$$
C = \pi(0.042)
$$

$$
\boxed{C \approx 0.132\text{ m}}
$$

The motors rotate at approximately:

$$
200\text{ RPM}
$$

Therefore, the theoretical linear velocity is:

$$
v = \frac{C \times RPM}{60}
$$

Substituting:

$$
v = \frac{0.132 \times 200}{60}
$$

$$
\boxed{v \approx 0.440\text{ m/s}}
$$

Converting to km/h:

$$
0.440 \times 3.6
$$

$$
\boxed{v \approx 1.58\text{ km/h}}
$$

Therefore, the theoretical no-load speed is approximately:

$$
\boxed{1.58\text{ km/h}}
$$

> **Important:** This is the theoretical no-load speed. The actual robot speed will normally be lower under load.

---

# 11. Check the Traction Limit

Torque alone does not determine how much force the robot can actually apply.

The tyres must be able to transfer the driving force to the WRO surface without slipping.

The maximum traction force can be estimated using:

$$
F_{traction} = \mu N
$$

where:

- $\mu$ = coefficient of friction between tyre and surface
- $N$ = normal force on the driven wheels

For example, if approximately **60% of the robot's weight** is supported by the rear driven wheels:

$$
N = 0.60 \times 12.75
$$

$$
N = 7.65\text{ N}
$$

If an example coefficient of friction is assumed:

$$
\mu = 0.8
$$

then:

$$
F_{traction} = 0.8 \times 7.65
$$

$$
\boxed{F_{traction} \approx 6.12\text{ N}}
$$

Our estimated driving force is:

$$
F_{drive} = 4.13\text{ N}
$$

Since:

$$
4.13 < 6.12
$$

the motor's estimated driving force would be below this example traction limit.

Therefore, under these assumptions, **motor torque is more likely to be the limiting factor than tyre traction during normal acceleration**.

> **Note:** The friction coefficient and rear weight distribution used here are estimates. They should not be presented as measured traction values.

---

# 12. Torque Distribution Through the Differential

The robot uses an **RC car rear differential**.

The two motors provide the combined input torque:

$$
\tau_{input} \approx 0.102\text{ N·m}
$$

After estimated drivetrain losses:

$$
\tau_{wheel} \approx 0.087\text{ N·m}
$$

During straight-line motion, the differential approximately distributes the available torque between the two rear wheels.

Assuming an equal torque distribution:

$$
\tau_{each\ wheel} = \frac{0.0867}{2}
$$

$$
\boxed{\tau_{each\ wheel} \approx 0.0434\text{ N·m}}
$$

The corresponding force per driven wheel is:

$$
F_{each} = \frac{0.0434}{0.021}
$$

$$
\boxed{F_{each} \approx 2.07\text{ N}}
$$

Therefore:

$$
2.07 + 2.07 \approx 4.14\text{ N}
$$

which agrees with the total driving-force calculation.

---

# 13. Summary of Calculated Values

| Quantity | Calculation | Result |
|---|---|---:|
| Wheel radius | $42/2$ | **21 mm** |
| Wheel radius | $21/1000$ | **0.021 m** |
| Torque per motor | $0.52 \times 0.0980665$ | **0.051 N·m** |
| Torque of 2 motors | $0.051 \times 2$ | **0.102 N·m** |
| Effective wheel torque | $0.102 \times 0.85$ | **0.087 N·m** |
| Driving force | $0.087/0.021$ | **4.13 N** |
| Robot weight | $1.3 \times 9.81$ | **12.75 N** |
| Idealized acceleration | $4.13/1.3$ | **3.18 m/s²** |
| Wheel circumference | $\pi \times 0.042$ | **0.132 m** |
| Theoretical speed | $0.132 \times 200/60$ | **0.440 m/s** |
| Theoretical speed | $0.440 \times 3.6$ | **1.58 km/h** |

---

# 14. Final Conclusion

> ### Drivetrain Torque Calculation
>
> The robot uses two N20 200 RPM geared motors to drive the rear axle through an RC car differential. The robot has a total mass of approximately **1.3 kg** and uses **42 mm diameter wheels**, giving a wheel radius of **0.021 m**.
>
> Assuming a rated motor torque of **0.52 kg·cm (0.051 N·m)** per motor, the combined rated motor torque is approximately **0.102 N·m**.
>
> Considering an estimated drivetrain efficiency of **85%**, approximately **0.087 N·m** of torque is available at the wheels.
>
> Using the wheel radius, this corresponds to an estimated theoretical driving force of approximately **4.13 N**.
>
> The theoretical wheel speed at 200 RPM is approximately **0.44 m/s (1.58 km/h)**.
>
> These calculations indicate that the drivetrain provides sufficient torque for the **1.3 kg robot under normal flat-surface operation**. However, actual performance will depend on tyre traction, battery voltage, motor loading, mechanical losses, and the exact torque-speed characteristics of the N20 motors.
>
> The calculated acceleration of **3.18 m/s²** should be treated as an idealized estimate rather than a measured value, since the motor's available torque changes with speed and load.

---

## 15. Important Engineering Considerations

The above calculations are useful for documenting the drivetrain design, but several values are based on assumptions.

### Motor Torque

The **0.52 kg·cm** value should be verified against the exact N20 motor datasheet or supplier specification. N20 motors are available with different gear ratios, voltages, speeds, and torque ratings.

### Drivetrain Efficiency

The assumed **85% efficiency** is an estimate. Actual efficiency depends on:

- Differential losses
- Bearing friction
- Gear alignment
- Axle friction
- Wheel friction
- Motor gearbox efficiency

### Battery Voltage

The theoretical calculations assume the motor operates close to its specified voltage. A 3S LiPo battery can have a significantly different voltage between full charge and discharge, which affects motor speed and torque.

### Real-World Speed

The calculated **1.58 km/h** is based on the nominal 200 RPM motor speed and 42 mm wheels. It is a theoretical value and does not account for load-dependent motor speed.

### Real-World Acceleration

The calculated **3.18 m/s²** should not be interpreted as the expected acceleration throughout the entire speed range. DC gear motors generally produce their highest torque at low speed, while available torque decreases as rotational speed increases.

---

## 16. Engineering Result

Based on the assumed motor specifications and drivetrain efficiency:

| Parameter | Result |
|---|---:|
| Total motor torque | **0.102 N·m** |
| Estimated wheel torque | **0.087 N·m** |
| Estimated driving force | **4.13 N** |
| Robot weight | **12.75 N** |
| Idealized acceleration | **3.18 m/s²** |
| Theoretical maximum speed | **0.440 m/s** |
| Theoretical maximum speed | **1.58 km/h** |

**Overall conclusion:** The calculated drivetrain torque is adequate for moving the 1.3 kg WRO robot on a flat competition surface under the stated assumptions. Further testing should be performed to determine the actual acceleration, maximum speed, traction limit, and drivetrain efficiency of the completed robot.


---

# Interactive Torque Calculator

The following interactive calculator allows the drivetrain parameters
to be changed dynamically.

### Parameters

- Motor torque
- Number of motors
- Wheel diameter
- Drivetrain efficiency
- Motor RPM

The calculator dynamically updates:

- Total motor torque
- Effective wheel torque
- Driving force
- Theoretical speed
- Torque-to-force visualisation

---

## Launch Calculator

🎛️ **[Open the Interactive Torque Calculator →](https://kiarabhandari8.github.io/Unsupervised-WROFE2026-India/docs/torque-calculator.html)**

> The calculator is hosted using GitHub Pages and runs directly in the browser.
$$
