# Talai-FRC-2025  
**FRC 2025 TimedRobot – Open-Source Robot Code (GPL-3.0)**  
Author: **Furkan Ege**

---

## 🧭 Overview
This repository contains the open-source robot code for the 2025 FRC season robot **Talai**.  
The project uses **WPILib 2025** and the **TimedRobot** programming model.

All derivatives must remain open-source under **GPL-3.0**.

---

## ⚙️ Technical Details
- **Language:** Java 17  
- **Framework:** WPILib 2025  
- **Architecture:** TimedRobot  
- **Build System:** GradleRIO  
- **Supported Hardware:**
  - RoboRIO 2.0  
  - SparkMAX (Brushed)  
  - navX  
  - USB Camera  
  - REVLib

---

## 📁 Project Structure
src/main/java/frc/robot/
│── Robot.java
│── Main.java

---

## 🚀 Deployment
To deploy the robot code:

1. Open the project in **WPILib VS Code**
2. Run:  
   `Ctrl + Shift + P → WPILib: Deploy Robot Code`
3. Ensure the RoboRIO is connected (USB/Ethernet/WiFi)
4. Deployment logs will confirm success

---

## 🔧 Build Without Deploying
Windows:
gradlew.bat build

Linux/macOS:
./gradlew build

---

## 🧠 Software Design Philosophy  
### *(How This Robot Works Perfectly Without Encoders or Limit Switches)*  

Talai’s elevator and mechanisms were built **without encoders or limit switches**.  
Despite this, the robot behaved *as if it had sensors*, thanks to carefully engineered software logic.

Below is the design philosophy that allowed the robot to operate safely, smoothly, and consistently.

---

### 🔹 1. Elevator Control Without Sensors
The elevator had:
- ❌ No encoders  
- ❌ No limit switches  
- ❌ No positional sensors  
- ❌ No physical brake  

Yet it behaved like a closed-loop system using these techniques:

#### **A. Software-Based "Bottom Limit Switch"**
Downward movement was **fully disabled** for safety.  
If the driver pressed LT, the code executed:

stopElevatorMotors();

This prevented:
- Gearbox damage  
- Rope slack  
- Backdriving  
- Mechanical stress at the bottom  

#### **B. Timed Upward Control (Virtual Encoder)**
Upward movement was based on:
- Driver input  
- Time-based soft limits  
- Controlled motor inversion  

In autonomous mode, a small fixed-duration lift simulated “reaching top”.

#### **C. Constant Hold Power (Virtual Brake)**
A small force was always applied to hold position:

HOLD_POWER = 0.30

This acted like a **software brake**, preventing gravity drop and oscillation.

---

### 🔹 2. Drive System — Motor Polarities Match Real Wiring  
One side of the drivetrain was wired backward on the physical robot.

Instead of rewiring motors, the code compensated:

leftPower = forward + rotation
rightPower = forward - rotation

This ensured:
- Straight driving  
- Predictable rotation  
- Smooth teleop feel  

---

### 🔹 3. Intake — Same Direction Motor Geometry  
Talai’s intake motors were physically mounted in a way that both needed to spin in the **same direction**.  
Therefore the code intentionally ran:

m_intakeLeft.set(0.55);
m_intakeRight.set(0.55);

This was not a mistake — it reflected the real mechanism.

---

### 🔹 4. Autonomous — 100% Timing Based  
With no sensors, autonomous relied on:
- A finite state machine  
- A shared timer  
- Carefully chosen durations  
- Fixed power control  

This made autonomous *repeatable and reliable*, despite zero feedback hardware.

---

### 🔹 Summary  
> *A robot can behave like it has sensors, even when it doesn’t —  
as long as the software is engineered correctly.*

Talai’s control system demonstrates how:
- Safety logic  
- Direction correction  
- Hold stabilization  
- Timing-based decisions  
- Motor synchronization  

can create a stable and high-performance robot without any encoder or limit switch hardware.

---

## 📜 License
This project is licensed under **GNU GPL-3.0**.  
Any modified versions *must* remain open-source.

Copyright (C) 2025 Furkan Ege

---

## 🤝 Contributions
Contributions, forks, and enhancements are welcome —  
as long as they comply with the GPL-3.0 copyleft terms.

---

## 📝 Notes
This project is fully independent of any team identity.  
All development rights belong to **Furkan Ege** as the original author.
