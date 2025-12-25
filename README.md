# EFD-Based Multi-Robot Formation Control

This repository contains the implementation of formation control strategies for multi-robot systems following Elliptic Fourier Descriptor (EFD) trajectories. The project transitions from theoretical simulations to real-world hardware implementation.

## 📌 Project Overview
The control algorithms are based on research by **Prof. Mustafa Ünel** and utilize both Implicit Polynomial (IP) for shape approach and Parametric EFD for trajectory maintenance.

### 1. Holonomic Simulation (2008 Paper)
Implementation of decentralized coordination and formation control for point-mass (holonomic) robots.

### 2. Non-Holonomic Simulation (2010 Paper)
Advanced tracking control using "virtual ghosts" to guide non-holonomic mobile robots along complex EFD curves while maintaining inter-robot coordination.

### 3. Real-Life Implementation (2025)
Hardware application on **TurtleBot3** platforms using:
- **ROS 2 (Humble)** for robot communication.
- **AprilTags (tag36h11)** for high-precision overhead vision localization.
- **MATLAB ROS 2 Toolbox** for centralized control and state estimation.

## 🛠️ Setup & Requirements
- **Software:** MATLAB (ROS 2 Toolbox, Image Processing Toolbox).
- **Robot Hardware:** TurtleBot3 (Burger/Waffle).
- **Vision:** Overhead camera for AprilTag detection (172mm tags).

## 🎓 Academic Background
This work was developed as part of my Master's thesis in Mechatronics at [Your University Name]. It serves as a bridge between advanced control theory and practical robotic swarms.
