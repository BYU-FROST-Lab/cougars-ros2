# 📦 Cougars ROS2 Package

This repository contains the ROS 2 packages, mission launch files, and scripts used by the BYU FRoSt lab's **Cooperative Underwater Group of Autonomous Robots (CoUGARs)** project.

---

## 🚀 Quick Start

For code development, you can clone this repository, create a new branch, and start working on your changes.

If you are setting up a new Coug-UV from scratch, please refer to the [main repository](https://github.com/BYU-FRoSt-Lab/cougars) for instructions on how to get the Docker image running.

All dependencies for this project are assumed to be available within the **`frostlab/cougars:vehicle`** Docker image (available on Docker Hub). This image provides a pre-configured environment with all necessary tools and libraries.

---

## Usage

Build ros2 workspace: `colcon build`

Run persistant launch:
```bash
ros2 launch cougars_bringup persistant_launch.py \
        namespace:=<namespace> \
        param_file:=<param_file> \
        fleet_param:=<fleet_param> \
        flags:=<flag1>:<value>,<flag2>:<value>
```

### Arguments

| Argument | Default | Description |
|---|---|---|
| `namespace` | `coug0` | ROS 2 namespace for the vehicle (e.g., `coug1`, `coug2`) |
| `param_file` | `/home/frostlab/config/deploy_tmp/vehicle_params.yaml` | Path to the vehicle-specific parameter YAML file |
| `fleet_param` | `/home/frostlab/config/deploy_tmp/fleet_params.yaml` | Path to the fleet-wide parameter YAML file |
| `flags` | _(empty)_ | Comma-separated `key:value` pairs forwarded as launch arguments to every child launch file |

### Flags

The `flags` argument is a comma-separated list of `key:value` pairs. Each pair is parsed and injected as an individual launch argument into all child launch files. A bare key with no value (e.g., `verbose`) is treated as `true`.

**Format:** `flags:=key1:value1,key2:value2,key3`

**Example:**
```bash
ros2 launch cougars_bringup persistant_launch.py \
        namespace:=coug1 \
        param_file:=/home/frostlab/config/deploy_tmp/vehicle_params.yaml \
        fleet_param:=/home/frostlab/config/deploy_tmp/fleet_params.yaml \
        flags:=sim:true,verbose,acoustic_pinger:true
```

#### Available Flags

| Flag | Default | Package | Description |
|---|---|---|---|
| `sim` | `false` | cougars_control | Run in simulation mode |
| `demo` | `false` | cougars_control | Run in demo mode |
| `verbose` | `false` | cougars_control | Enable verbose (`screen`) output from control nodes |
| `fins_manual` | `false` | cougars_control | When `true`, replaces `coug_controls` with `fins_manual.py` for direct fin control |
| `manual_mission` | `true` | cougars_control | When `true`, launches `manual_mission.py` in place of the waypoint follower |
| `acoustic_pinger` | `false` | cougars_coms | When `true`, launches the vehicle acoustic pinger (sends acoustic messages on repeat) |
| `debug` | `false` | cougars_coms | When `true`, enables debug logging for the RF bridge |
| `use_dvl` | `true` | cougars_bringup | When `false`, disables the DVL sensor and manager nodes |
| `use_gps` | `true` | cougars_bringup | When `false`, disables the GPS nodes |
| `acoms_on` | `true` | cougars_bringup | When `false`, disables the Seatrac acoustic modem node |


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
