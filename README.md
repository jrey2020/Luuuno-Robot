# 🤖 Luuno Robot

**Prepared by:** Jaime Rey  
**Major:** Computer / Software Engineering  
**Date:** July 30, 2025  

Luuno is a fully autonomous AI-powered robot designed and built from scratch, integrating advanced sensors, cameras, and custom software to perform complex navigation and interaction tasks without human intervention.  
This project demonstrates complete robotics system design: hardware, electronics, software, and real-world deployment.

---

## 🛠 Hardware Components

- **Jetson Orin Nano** – High-performance AI edge computer running JetPack 6.1 for computer vision and deep learning.
- **Arduino Leonardo** – Microcontroller for precise I/O and USB emulation.
- **ZED 2i Stereo Camera** – Depth mapping, SLAM, and face recognition.
- **Intel RealSense D455** – Depth sensing and gesture detection.
- **RPLIDAR S3** – 2D LiDAR for mapping and obstacle detection.
- **Adafruit BNO055** – 9-DOF orientation sensor with sensor fusion.
- **RoboClaw 2x15A** – Smart motor controller with encoder support.
- **80mm Mecanum Wheels with 12V Gear Motors** – Omni-directional movement.
- **TP-Link Powered USB Hub** – Stable USB connectivity for multiple sensors.
- **Eyoyo 7" Mini Monitor** – Compact on-board display.
- **Zeee 3S LiPo Battery (5200mAh, 11.1V)** – Main power source.

Detailed wiring diagrams, CAD models, and schematics are in [`/hardware`](hardware).

---

## 💻 Software Overview

### **Program 1 — Gesture + Face Tracking Control**
- Switches between gesture control and face-following using RealSense D455 and ZED 2i.
- Touchless control via hand gestures; follows recognized user in face mode.
- Real-time multi-threaded architecture for smooth operation.

### **Program 2 — Vision-Guided Target Tracking**
- Detects and approaches specific **ArUco markers** using ZED camera depth and RoboClaw motor control.
- Ideal for warehouse navigation and visual docking.

### **Program 3 — Precision Encoder & LIDAR Navigation**
- Executes precise turns, parking, and obstacle avoidance using LIDAR + encoders.
- Fully autonomous without requiring AI or heavy compute.

### **Program 4 — Hybrid Escape Navigation**
- Solves maze-like environments with hybrid sensing: LIDAR + ZED EXIT detection + encoders.
- Avoids loops, dynamically selects best path, and parks at exit.

Source code for all programs is in [`/code`](code).

---

## 📄 Documentation
- Full project report in [`/docs`](docs)  
  Includes:
  - Component datasheets
  - Assembly instructions
  - Software setup
  - Test results and analysis

---

## 📸 Media
Photos and videos of Luuno in action are in [`/media`](media).

---

## 🚀 How to Run

1. **Clone this repository**
   ```bash
   git clone https://github.com/jrey2020/Luuuno-Robot.git
   cd Luuuno-Robot/code
