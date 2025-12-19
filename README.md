# 📦 Cougars ROS2 Package

This repository contains the ROS 2 packages, mission launch files, and scripts used by the BYU FRoSt lab's **Cooperative Underwater Group of Autonomous Robots (CoUGARs)** project.

---

## 🚀 Quick Start

For code development, you can clone this repository, create a new branch, and start working on your changes.

If you are setting up a new Coug-UV from scratch, please refer to the [main repository](https://github.com/BYU-FRoSt-Lab/cougars) for instructions on how to get the Docker image running.

All dependencies for this project are assumed to be available within the **`frostlab/cougars:vehicle`** Docker image (available on Docker Hub). This image provides a pre-configured environment with all necessary tools and libraries.

---

## 📂 High-Level Overview of Packages

* **⚙️ cougars\_control**
  Core control nodes for the vehicle:

  * PID controllers for depth, heading, and pitch (`coug_controls.cpp`)
  * Kinematics node for thruster trim & fin offsets (`coug_kinematics.cpp`)
  * Mission-related scripts (`waypoint_follower.cpp`, `manual_mission.py`)

* **📡 cougars\_localization**
  Localization and state estimation:

  * Sensor data converters (`depth_convertor.cpp`, `dvl_convertor.cpp`, `seatrac_ahrs_convertor.cpp`)
  * Factor graph-based state estimator (`factor_graph.py`)
  * Static TF broadcaster (`static_tf_publisher.cpp`)

* **🌐 cougars\_coms**
  Handles acoustic modem & RF communications:

  * Acoustic messaging between vehicles (`cougars_coms.cpp`)
  * RF bridge for base station comms (`rf_bridge.py`)

* **🧩 cougars\_interfaces**
  Defines custom ROS 2 messages and services:

* **🚦 cougars\_bringup**
  Launch files and system bringup scripts:

  * Persistent launch manager (`persistant_launch.py`)
  * Automatic rosbag recorder (`bag_recorder.cpp`)

---

## 📝 General Notes

* **🔄 Continuous Integration**: GitHub Actions workflow (`colcon_build_test.yml`) automatically builds & tests packages in Docker.
* **📜 Licensing**: Apache License 2.0 for most packages. Provides rights to use, modify, and distribute with attribution and license notice.
