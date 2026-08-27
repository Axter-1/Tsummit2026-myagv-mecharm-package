# myAGV Home Service

ROS 2 Humble project for an Elephant Robotics myAGV + MechArm 270 M5.

## Current baseline

- Gazebo Classic simulation
- Mecanum motion
- MechArm trajectory controller
- Camera
- LiDAR
- ArUco detection
- Geometric ArUco alignment
- LiDAR-assisted final approach
- SLAM / saved map
- Nav2 + AMCL
- twist_mux arbitration
- YAML mission manager
- Finite or infinite mission repetition

## Supported development environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

x86_64 and compatible ARM64 Ubuntu installations can be used as long as
all ROS dependencies required by the project are available.

## Clone

```bash
git clone <PRIVATE_REPOSITORY_URL> myagv_home_service_ws
cd myagv_home_service_ws
./scripts/bootstrap.sh
source scripts/env.sh
./scripts/build.sh
./scripts/run_nav2_sim.sh
./scripts/run_mission_sim.sh
repeat: 1
Mission repetition

Inside the mission YAML:

repeat: 1

runs once.

repeat: 5

runs five times.

repeat: 0

runs indefinitely until interrupted.
