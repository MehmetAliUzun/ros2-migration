# ROS2 Migration – Bachelor Thesis Project

This repository contains the development workspace and source code for a bachelor thesis project focused on migrating a ROS1-based robotic system to ROS2. The system is designed for the **Transformation Hub Leitungssatz Challenge**, which involves automating the handling and insertion of flexible wiring harnesses using a UR5e cobot.

## 🧠 Project Goal

The primary goal is to **reimplement the existing ROS1 system in ROS2 (Humble Hawksbill)**, ensuring functional equivalence and improved maintainability. This includes:

- Understanding and analyzing the original ROS1 codebase
- Modeling the robot work environment in simulation
- Rewriting system nodes in ROS2 using modern conventions
- Testing the migrated system both in simulation and on hardware

## 🛠️ Workspace Structure

This repository is a ROS2 workspace (`bachThes_ws`) with the following layout:

ls_ros2/
├── src/ # ROS2 packages will go here
├── install/ # (generated) ROS2 install files
├── build/ # (generated) build files
├── log/ # (generated) logs



> Note: `install/`, `build/`, and `log/` are ignored via `.gitignore`


📦 Current Status
✅ ROS2 workspace initialized and synced with GitHub

⏳ ROS1 codebase under analysis

⏳ Simulation environment being developed in Gazebo

⏳ ROS2 node reimplementation pending

📚 License
This project is licensed under the MIT License. See the LICENSE file for details.
