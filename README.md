# PX4 Drone Development - ROS 2 Autonomous Flight Stack

This repository contains a Rust-based PX4 autonomous flight control system integrated with ROS 2. It supports running in a Docker environment with full simulation and communication bridging via Micro XRCE-DDS Agent. The system implements autonomous takeoff, waypoint navigation, return-to-launch (RTL), safe landing, and telemetry logging.

---

## Features

- Autonomous mission planning with a square waypoint pattern
- Fully autonomous control: all arming and mode changes done within the node
- State machine implementation for mission phases: takeoff, hold, waypoint flight, RTL, land, disarm
- Basic failsafe for pose validity and system flags that triggers safe landing (unstable)
- Telemetry logging to console and CSV for post-flight analysis
- ROS 2 node designed to run inside Docker for environment consistency

---

## Getting Started

### Clone the repository

git clone https://github.com/Abhi-creator/PX4_Drone_Development.git
cd PX4_Drone_Development


### Build Docker environment

Build the Docker image using the provided Dockerfile:


####Or pull prebuilt image (recommended as lots of dependencies were only added in image):

####docker pull agentabhi/px4-ros2-rust:latest


### Setup Micro XRCE-DDS Agent (not needed if using prebuild image)

The Micro XRCE-DDS Agent needs to be built and installed separately as follows:
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent

Edit `CMakeLists.txt` to update Fast-CDR version:
set(_fastcdr_version 2.13.0)
set(_fastcdr_tag 2.13.x)

mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/


### Install other dependencies

clone px4_msg and ros2-rust inside rust_workspace/src

---

## Running the Simulation and Node

Launch the Docker container with proper environment variables for GUI forwarding if needed:


---

## Running the System

Open multiple terminals (inside the container):

1. **Run Micro XRCE Agent**

MicroXRCEAgent udp4 -p 8888


2. **Launch PX4 SITL Simulator**

make px4_sitl gz_x500


3. **Run ROS2 Node**

cd rust_workspace
source install/setup.bash
ros2 run px4_fsm px4_fsm


Make sure to execute

source source_env.sh


in each terminal to set up environment variables correctly.

---

## Mission Overview

- Continuous Offboard commands sent at 10 Hz.
- Waits for valid pose before starting mission.
- Automatically switches to OFFBOARD mode and arms.
- Executes square waypoint flight.
- Returns home, lands, and disarms autonomously.
- Logs telemetry for post-flight analysis.

---

