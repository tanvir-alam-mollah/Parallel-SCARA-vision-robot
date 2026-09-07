# Autonomous 5-Bar Parallel SCARA Manipulator
**Developer:** Md Tanvir Alam Mollah, Mechatronics Engineering, KUET  
**Domain:** Autonomous Systems Integration & Closed-Loop Robotics  
<img width="950" height="726" alt="3" src="https://github.com/user-attachments/assets/1c513f9c-bccf-4ef2-a514-d2dce302a55d" />
![Project Status](https://img.shields.io/badge/Status-Active_Development-brightgreen)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Language](https://img.shields.io/badge/Language-C++_%7C_Python-yellow)



## 📌 Project Overview
A direct-drive, 2-DOF parallel robotic manipulator designed for fully autonomous pick-and-place operations. This project bridges advanced mechanical kinematics with real-time computer vision, allowing the system to identify targets within a calibrated workspace, compute real-world cartesian coordinates, and execute precision sorting using a custom electromagnet end-effector. 

To guarantee absolute physical precision, the system incorporates a closed-loop feedback architecture. By utilizing AS5600 magnetic encoders to count the physical steps of the stepper motors, the microcontroller continuously verifies the true position of the end-effector against the calculated kinematic targets.

## ✨ Key Features
*   **Dynamic Inverse Kinematics (IK):** Mathematical IK natively derived and executed on the ESP32 to calculate complex joint angles in real time.
*   **Closed-Loop Position Feedback:** Dual AS5600 magnetic encoders count the physical steps executed by the NEMA 17 motors, verifying the exact position of the end-effector and forming a precise error-correction feedback loop.
*   **Computer Vision Pipeline:** OpenCV-based object detection over an IP camera stream, transforming pixel data into real-world cartesian coordinates (0.875 mm/px scale).
*   **Visual Auto-Homing (`VHOME`):** The system dynamically calibrates its starting coordinate by visually locating a fiducial marker on the end-effector at startup, eliminating the need for hardcoded home steps.
*   **Handshake Protocol Automation:** A robust serial communication architecture between Python and the ESP32 that orchestrates the full autonomous task loop (Identify $\rightarrow$ Grasp $\rightarrow$ Transport $\rightarrow$ Release $\rightarrow$ Reset).
*   **Live UI/UX Overlay:** Projects a persistent 100mm physical coordinate grid directly onto the video feed to monitor the manipulator's reach and spatial accuracy continuously.

## 🛠️ Hardware Architecture
*   **Microcontroller:** ESP32 DevKit V1
*   **Actuators:** 2x NEMA 17 Stepper Motors (Direct Drive) driven by DRV8825 modules (1/32 Microstepping).
*   **Position Feedback:** 2x AS5600 High-Resolution Magnetic Encoders operating on independent hardware I2C buses.
*   **End-Effector:** 5V DC Electromagnet (P25/20, 5kg holding capacity, 0.6A draw) controlled via a 1-Channel 5V Relay.
*   **Logic Level Management:** 3.3V to 5V relay mismatch resolved via software-defined High-Impedance switching (`pinMode` toggling).

## 📐 Mechanical Workspace & Kinematics
*   **Base Origins:** Left Motor (X: -40.0, Y: 0.0) | Right Motor (X: 40.0, Y: 0.0)
*   **Linkages:** Base Links (110.0 mm) | End Links (160.0 mm)
*   **Trajectory:** Uses `FastAccelStepper` for interrupt-driven trapezoidal motion profiling (Acceleration: 2000 steps/s²).

## 🚀 Current Status & Next Steps
The manipulator's kinematics, vision pipeline, and open-loop control are fully operational.

**Active Development:** Implementing the closed-loop feedback system. Dual AS5600 magnetic encoders are being mapped via independent hardware I2C buses to count the exact physical steps of the NEMA 17 motors. This feedback loop will continuously verify the mechanical execution of the Inverse Kinematics, ensuring the end-effector has successfully reached the assigned coordinates and compensating for any missed stepper motor steps.

---

### 📂 Repository Structure
*   `/firmware` - ESP32 C++ source code, kinematics math, and stepper control logic.
*   `/vision` - Python scripts for OpenCV target tracking, auto-homing, and serial communication.
*   `/docs` - Wiring diagrams, mechanical parameters, and system architecture notes.
*   `/cad` - 3D models and assembly files for the 5-bar linkage system.
<img width="1120" height="720" alt="2" src="https://github.com/user-attachments/assets/3dfd1169-ca83-4233-9dee-bbe2749f705c" />
<img width="1336" height="816" alt="1" src="https://github.com/user-attachments/assets/85922f80-7282-4461-8242-8778ed714fcd" />
