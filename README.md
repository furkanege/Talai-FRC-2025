# Talai-FRC-2025  
**FRC 2025 TimedRobot – Open-Source Robot Code (GPL-3.0)**  
Author: **Furkan Ege**

## 🧭 Overview
This repository contains the open-source robot code for the 2025 FRC season robot **Talai**.  
The project uses **WPILib 2025** and the **TimedRobot** programming model.

All derivatives must remain open-source under **GPL-3.0**.

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

## 📁 Project Structure
src/main/java/frc/robot/
│── Robot.java
│── Main.java

## 🚀 Deployment
To deploy the robot code:

1. Open the project in **WPILib VS Code**
2. Run: Ctrl + Shift + P → WPILib: Deploy Robot Code
3. Ensure the RoboRIO is connected (USB/Ethernet/WiFi)
4. Deployment logs will confirm success

## 🔧 Build Without Deploying
Windows:
gradlew.bat build

Linux/macOS:
./gradlew build

## 📜 License
This project is licensed under **GNU GPL-3.0**.  
Any modified versions *must* remain open-source.

Copyright (C) 2025 Furkan Ege

## 🤝 Contributions
Contributions, forks, and enhancements are welcome —  
as long as they comply with the GPL-3.0 copyleft terms.

## 📝 Notes
This project is fully independent of any team identity.  
All development rights belong to **Furkan Ege** as the original author.
