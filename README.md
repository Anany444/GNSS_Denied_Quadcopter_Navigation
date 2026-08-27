<!--
<table>
  <tr>
    <th colspan="1">RGB-D Visual Odometry (VO) Setup</th>
    <th colspan="1">LiDAR–Inertial Odometry (LIO) Setup</th>
  </tr>
  <tr>
    <td><img width="600" alt="VO Setup 1" src="https://github.com/user-attachments/assets/c06bb840-6a56-42ca-8dbe-0fac94f51f6c" /></td>
    <!-- <td><img width="300" alt="VO Setup 2" src="https://github.com/user-attachments/assets/eb1fb727-8c16-4a09-9109-b5b66fadec56" /></td> -->
   <!-- <td><img width="600" alt="LIO Setup 1" src="https://github.com/user-attachments/assets/eb2de49b-1fc1-4af1-84fc-6a2b2643e2d8" /></td> -->
   <!-- <td><img width="300" alt="LIO Setup 2" src="https://github.com/user-attachments/assets/37810619-905d-4d35-a2b7-1f96a9115ec8" /></td> -->
<!--  </tr>
</table>

<img width="400" height="400" alt="lio1_gif4" src="https://github.com/user-attachments/assets/3dcd0194-49f0-4ade-991f-e6dfda023098" />

<img width="900" height="362" alt="outdoor_map" src="https://github.com/user-attachments/assets/aa9b1b53-225e-434d-acb7-33786b49a3f1" />

<img width="1150" height="354" alt="outdoor_map2" src="https://github.com/user-attachments/assets/b8f0c1a0-3e87-4714-9ace-ba185fd7a1a9" />

<video
  src="https://github.com/user-attachments/assets/af495c0a-7ccb-462e-bc85-170269777214"
  width="5%"
  controls>
</video>

<video
  src="https://github.com/user-attachments/assets/e615a37a-06e0-4dc8-9170-f577e0f83028"
  width="5%"
  controls>
</video> -->
# 🚁 GNSS-Denied Quadcopter Navigation

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-Autopilot-purple?logo=px4&logoColor=white)
![RTAB-Map](https://img.shields.io/badge/RTAB--Map-Visual%20SLAM-green)
![Point-LIO](https://img.shields.io/badge/Point--LIO-LiDAR%20Inertial-orange)
![Jetson Orin Nano](https://img.shields.io/badge/NVIDIA-Jetson%20Orin%20Nano-76B900?logo=nvidia&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-yellow)

A GNSS-denied navigation system for a quadcopter using two independent onboard localization approaches, **RGB-D Visual Odometry (VO)** and **LiDAR–Inertial Odometry (LIO)**.

<table>
  <tr>
    <td align="center">
      <img width="400" alt="Visual Odometry Setup" src="https://github.com/user-attachments/assets/c06bb840-6a56-42ca-8dbe-0fac94f51f6c" />
    </td>
    <td align="center">
      <img width="400" alt="LiDAR–Inertial Odometry Setup" src="https://github.com/user-attachments/assets/eb2de49b-1fc1-4af1-84fc-6a2b2643e2d8" />
    </td>
  </tr>
  <tr>
    <th>Visual Odometry Setup</th>
    <th>LiDAR–Inertial Odometry Setup</th>
  </tr>
</table>

---

## 📖 Overview

GNSS is widely used for UAV localization, but its availability and reliability can be compromised in GNSS-denied environments, including areas affected by **GNSS jamming or spoofing**, as well as indoor and obstructed environments. This project explores onboard perception as an alternative source of localization for a quadcopter.

Two independent localization pipelines were implemented on the same platform:

| Pipeline | Sensor | Processing |
|---|---|---|
| **RGB-D VO** | Intel RealSense D455F | RTAB-Map RGB-D Odometry + SLAM |
| **LIO** | Unitree L2 3D LiDAR + IMU | Point-LIO |

Both pipelines run on an **NVIDIA Jetson Orin Nano** companion computer running **ROS 2 Jazzy** and provide external odometry to the **Cube Orange+** flight controller running **PX4** through the **uXRCE-DDS** bridge over a serial connection.

### 🏠 GNSS-denied indoor position hold
>⏳ **Please wait a few seconds for the GIF to load/render**

<p align="left">
  <img width="500" alt="GNSS Denied Indoor Position Hold" src="https://github.com/user-attachments/assets/3dcd0194-49f0-4ade-991f-e6dfda023098" />
  <br><em> </em>
</p>


<!-- <img width="400" height="400" alt="GNSS Denied Indoor Position Hold" src="https://github.com/user-attachments/assets/3dcd0194-49f0-4ade-991f-e6dfda023098" /> -->

---

## 📑 Table of Contents

- [Overview](#overview)
- [Demo](#demo)
  - [RGB-D Visual Odometry](#rgb-d-visual-odometry)
  - [LiDAR–Inertial Odometry](#lidar-inertial-odometry)
  - [3D LiDAR Mapping](#3d-lidar-mapping)
- [Platform & Software](#platform--software)
- [System Architecture](#system-architecture)
- [Localization Approaches](#localization-approaches)
  - [RGB-D Visual Odometry (VO)](#rgb-d-visual-odometry-vo)
  - [LiDAR–Inertial Odometry (LIO)](#lidar-inertial-odometry-lio)
- [PX4 Integration](#px4-integration)
- [Repository Structure](#repository-structure)
- [Limitations](#limitations)
- [Future Work](#future-work)
---

## 🎬 Demo

### 📷 RGB-D Visual Odometry
> ▶️ **Click play to start the demo.** <video
  src="https://github.com/user-attachments/assets/e615a37a-06e0-4dc8-9170-f577e0f83028"
  width="640"
  controls>
</video>

### 📡 LiDAR Inertial Odometry
> ▶️ **Click play to start the demo.**<video
  src="https://github.com/user-attachments/assets/af495c0a-7ccb-462e-bc85-170269777214"
  width="640"
  controls>
</video>

### 🗺️ 3D LiDAR Mapping

> ⏳ **Please wait a few seconds for the GIF to load/render**

<p align="center">
  <img width="900" alt="Outdoor 3D point cloud map — Location 1" src="https://github.com/user-attachments/assets/aa9b1b53-225e-434d-acb7-33786b49a3f1" />
  <br><em>Outdoor 3D point cloud map with path followed by drone in purple — Location 1</em>
</p>

<p align="center">
  <img width="1150" alt="Outdoor 3D point cloud map — Location 2" src="https://github.com/user-attachments/assets/b8f0c1a0-3e87-4714-9ace-ba185fd7a1a9" />
  <br><em>Outdoor 3D point cloud map with path followed by drone in purple — Location 2</em>
</p>

---

## 🖥️ Platform & Software

The two approaches use the same quadcopter platform and companion computer, with different perception sensors and localization software.

<table>
  <tr>
    <th colspan="1">RGB-D Visual Odometry (VO)</th>
    <th colspan="1">LiDAR–Inertial Odometry (LIO)</th>
  </tr>
  <tr>
    <td><img width="400" alt="VO Setup 1" src="https://github.com/user-attachments/assets/c06bb840-6a56-42ca-8dbe-0fac94f51f6c" /></td>
    <td><img width="400" alt="LIO Setup 1" src="https://github.com/user-attachments/assets/eb2de49b-1fc1-4af1-84fc-6a2b2643e2d8" /></td>
  </tr>
  <tr>
    <td colspan="1">

**Hardware**
- Intel RealSense D455F
- NVIDIA Jetson Orin Nano
- Cube Orange+
- Holybro X500 V2


**Software**
- ROS 2 Jazzy
- RTAB-Map
- uXRCE-DDS
- PX4 Autopilot 
    </td>
    <td colspan="2">

**Hardware**
- Unitree L2 3D LiDAR
- NVIDIA Jetson Orin Nano
- Cube Orange+
- Holybro X500 V2

**Software**
- ROS 2 Jazzy
- Point-LIO
- uXRCE-DDS
- PX4 Autopilot
    </td>
  </tr>
</table>

---

## 🏗️ System Architecture

The two pipelines share the same flight controller integration but use different sensors and odometry algorithms. They are **independent alternatives** — not fused together.

<img width="1672" height="941" alt="image" src="https://github.com/user-attachments/assets/4053be98-fe3b-4b2c-86cd-6e9093e4c3e0" />
<!--
```text
╔══════════════════════════════════════╗   ╔══════════════════════════════════════╗
║         VISION PIPELINE (VO)         ║   ║          LiDAR PIPELINE (LIO)        ║
╠══════════════════════════════════════╣   ╠══════════════════════════════════════╣
║                                      ║   ║                                      ║
║   ┌─────────────────────────────┐    ║   ║   ┌─────────────────────────────┐    ║
║   │     Intel RealSense D455F   │    ║   ║   │       Unitree L2 LiDAR      │    ║
║   │  RGB stream + Depth stream  │    ║   ║   │   3D Point Cloud  +  IMU    │    ║
║   └────────────┬────────────────┘    ║   ║   └────────────┬────────────────┘    ║
║                │  USB 3.0            ║   ║                │  Ethernet           ║
║                ▼                     ║   ║                ▼                     ║
║   ┌─────────────────────────────┐    ║   ║   ┌─────────────────────────────┐    ║
║   │     Jetson Orin Nano        │    ║   ║   │     Jetson Orin Nano        │    ║
║   │       ROS 2 Jazzy           │    ║   ║   │       ROS 2 Jazzy           │    ║
║   │                             │    ║   ║   │                             │    ║
║   │  ┌───────────────────────┐  │    ║   ║   │  ┌───────────────────────┐  │    ║
║   │  │      RTAB-Map         │  │    ║   ║   │  │       Point-LIO       │  │    ║
║   │  │  RGB-D VO + SLAM      │  │    ║   ║   │  │  LiDAR–Inertial Odom  │  │    ║
║   │  └──────────┬────────────┘  │    ║   ║   │  └──────────┬────────────┘  │    ║
║   │             │ /rtabmap/odom │    ║   ║   │             │ /odom_correc- │    ║
║   │             │  (ENU / ROS)  │    ║   ║   │             │  ted (ENU/ROS)│    ║
║   │             ▼               │    ║   ║   │             ▼               │    ║
║   │  ┌───────────────────────┐  │    ║   ║   │  ┌───────────────────────┐  │    ║
║   │  │    vision_bridge      │  │    ║   ║   │  │    lidar_bridge       │  │    ║
║   │  │  • ENU→NED frame conv │  │    ║   ║   │  │  • ENU→NED frame conv │  │    ║
║   │  │  • nav_msgs/Odometry  │  │    ║   ║   │  │  • nav_msgs/Odometry  │  │    ║
║   │  │    → px4_msgs/        │  │    ║   ║   │  │    → px4_msgs/        │  │    ║
║   │  │    VehicleOdometry    │  │    ║   ║   │  │    VehicleOdometry    │  │    ║
║   │  └──────────┬────────────┘  │    ║   ║   │  └──────────┬────────────┘  │    ║
║   │             │ /fmu/in/      │    ║   ║   │             │ /fmu/in/      │    ║
║   │             │ vehicle_      │    ║   ║   │             │ vehicle_      │    ║
║   │             │ visual_odom   │    ║   ║   │             │ visual_odom   │    ║
║   │             ▼               │    ║   ║   │             ▼               │    ║
║   │  ┌───────────────────────┐  │    ║   ║   │  ┌───────────────────────┐  │    ║
║   │  │   uXRCE-DDS Agent     │  │    ║   ║   │  │   uXRCE-DDS Agent     │  │    ║
║   │  │  (MicroXRCEAgent)     │  │    ║   ║   │  │  (MicroXRCEAgent)     │  │    ║
║   │  └──────────┬────────────┘  │    ║   ║   │  └──────────┬────────────┘  │    ║
║   └─────────────│───────────────┘    ║   ║   └─────────────│───────────────┘    ║
║                 │  Serial UART       ║   ║                 │  Serial UART       ║
║                 │  921600 baud       ║   ║                 │  921600 baud       ║
╚═════════════════│════════════════════╝   ╚═════════════════│════════════════════╝
                  │                                           │
                  └─────────────────┬─────────────────────────┘
                                    ▼
                  ┌─────────────────────────────────────────┐
                  │           Cube Orange+  /  PX4           │
                  │                                          │
                  │   uXRCE-DDS Client  ──▶  uORB bus        │
                  │                              │           │
                  │                             EKF2         │
                  │                    (sensor fusion /      │
                  │                    GNSS-denied nav)      │
                  └──────────────────────────────────────────┘
```
-->


---

## 🧭 Localization Approaches

### 📷 RGB-D Visual Odometry (VO)

An **Intel RealSense D455F** provides RGB and depth measurements to RTAB-Map.

```text
D455F
 │
 ├── RGB
 ├── Depth
 └── Camera Info
       │
       ▼
 RTAB-Map RGB-D Odometry
       │
       ├── Visual Odometry
       │
       └── SLAM
            ├── Loop Closure
            └── Pose Graph Optimization
       │
       ▼
 External Odometry
       │
       ▼
      PX4
```


### 📡 LiDAR Inertial Odometry (LIO)

The second approach uses a **Unitree L2 3D LiDAR** and its inertial measurements with Point-LIO.

```text
Unitree L2
 │
 ├── 3D LiDAR
 └── IMU
       │
       ▼
    Point-LIO
       │
       ├── LiDAR–Inertial Odometry
       └── 3D Point Cloud Map
       │
       ▼
 External Odometry
       │
       ▼
      PX4
```

---

## 🔗 PX4 Integration

Both localization pipelines run on the Jetson Orin Nano and communicate with the **Cube Orange+ running PX4** through the **uXRCE-DDS** middleware bridge over a serial connection (UART @ 921600 baud).

### 🔌 Agent / Client Architecture

| Component | Runs on | Role |
|---|---|---|
| **uXRCE-DDS Agent** (`MicroXRCEAgent`) | Jetson — ROS 2 Jazzy | Bridges ROS 2 ↔ PX4 uORB over serial |
| **uXRCE-DDS Client** | Cube Orange+ — PX4 firmware | Exposes uORB topics to the agent |

The agent transparently exposes PX4's internal **uORB topics** as typed **ROS 2 messages** (`px4_msgs`), enabling bidirectional communication — the Jetson can publish setpoints to PX4 and subscribe to PX4 telemetry, all within the standard ROS 2 ecosystem.

#### Required packages (Jetson side)

```bash
# 1. Micro XRCE-DDS Agent
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install

# 2. PX4 ROS 2 message definitions
# Clone px4_msgs into your ROS 2 workspace src/ and build
git clone https://github.com/PX4/px4_msgs.git
colcon build --packages-select px4_msgs
```

### 📤 External Odometry Topic

Odometry estimated by either VO (`vision_bridge`) or LIO (`lidar_bridge`) is published to:

```
/fmu/in/vehicle_visual_odometry   [px4_msgs/msg/VehicleOdometry]
```

Both bridge nodes perform the necessary **ENU (ROS) → NED (PX4)** coordinate frame conversion and `nav_msgs/Odometry` → `px4_msgs/VehicleOdometry` message conversion before publishing.

### ⚙️ PX4 EKF2 Parameter Setup

To enable GNSS-denied navigation using external odometry, the following EKF2 parameters must be configured on the flight controller:

| Parameter | Description |
|---|---|
| `EKF2_EV_CTRL` | Bitmask controlling which external vision (EV) measurements the EKF2 fuses — position, velocity, yaw, and/or height |
| `EKF2_HGT_REF` | Selects the primary altitude reference source used by EKF2 — set to EV to use odometry height instead of barometer or GPS |
| `EKF2_EV_DELAY` | Compensates for the timestamp offset between when odometry is generated on the Jetson and when EKF2 processes it |
| `EKF2_EV_NOISE_MD` | Controls whether EKF2 uses fixed noise values or reads the covariance directly from the incoming odometry message |
| `EKF2_GPS_CTRL` | Bitmask enabling/disabling GPS measurement fusion in EKF2 — disable when flying GNSS-denied |
| `EKF2_BARO_CTRL` | Enables or disables barometer fusion as a height source in EKF2 |



---

## 📂 Repository Structure

```text
gnss-denied-quadcopter-navigation/
│
├── README.md
│
├── vo/
│   ├── launch/
│   ├── config/
│   └── scripts/
│
├── lio/
│   ├── launch/
│   ├── config/
│   └── scripts/
│
├── px4/
│   └── config/
│
└── media/
    ├── images/
    └── videos/
```

---

## 🔮 Future Work

- Resolve D455F IMU compatibility with jetpack version, and evaluate RGB-D VIO.
- Quantitatively evaluate VO and LIO against ground truth.
- Investigate visual–LiDAR fusion.
- Explore GPU accelerated approaches for both VIO and LIO to exploit the full potential of Jetson Orin Nano.
- Implement frontier based autonomous exploration.
