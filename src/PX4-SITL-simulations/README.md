# PX4 SITL (Gazebo Classic) — Install & Build (ROS1 + MAVROS)

This README explains how to **install dependencies and build PX4 SITL** for Gazebo **Classic** with ROS1/MAVROS.  
Once the build succeeds, run **your own startup script** (e.g., `scripts/start_sitl.sh`).

> **Note (OS support):** Gazebo **Classic** is the legacy simulator. PX4 now officially supports the new **Gazebo (gz)** on Ubuntu 22.04+; Gazebo Classic is best used on Ubuntu **20.04**. See “Official References”. :contentReference[oaicite:0]{index=0}

---

## 0) Recommended environment

- **Ubuntu 20.04** (ROS Noetic + Gazebo Classic + MAVROS)  
- If you use **Ubuntu 22.04+**, prefer the new **Gazebo (gz)** instead of Classic (this README targets Classic). :contentReference[oaicite:1]{index=1}

---

## 1) Get PX4 source & set up toolchain

```bash
# Essentials
sudo apt update
sudo apt install -y git

# Clone PX4 (use --recursive)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Run the official setup script for Ubuntu
bash ./Tools/setup/ubuntu.sh
```

## 2) Install ROS1 + MAVROS (for ROS topics and tools)

```bash

# ROS Desktop (skip if already installed)
sudo apt install -y ros-noetic-desktop-full

# MAVROS + extras
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras

# GeographicLib datasets (required by MAVROS)
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# Auto-source ROS on new terminals
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 3) Build PX4 for Gazebo Classic (without running)
```bash
cd ~/PX4-Autopilot   # ensure you're at the repo root
DONT_RUN=1 make px4_sitl gazebo-classic

```

## 4) Run startup script
```bash
chmod +x ~./start_sitl.sh
~./start_sitl.sh
```

## 5) Control the AprilTag-equipped rover
```bash
# 10 Hz: forward 0.5 m/s, yaw left 0.3 rad/s
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
"{ linear:  {x: 0.5, y: 0.0, z: 0.0},
   angular: {x: 0.0, y: 0.0, z: 0.3} }"
```
