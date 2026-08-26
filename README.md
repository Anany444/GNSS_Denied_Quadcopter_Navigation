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

### 🏠 GNSS Denied Indoor Position Hold
> ▶️ **Click play to start the demo.** <video
  src="https://github.com/user-attachments/assets/72488990-236b-4f18-a3b2-daf8a11ab9ca"
  width="140"
  autoplay
  loop
  muted
  playsinline
  controls>
</video>


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

> ⏳ **Please wait a few seconds for the GIF to load/render, then ▶️ click play to start the demo.**

<p align="center">
  <img width="900" alt="Outdoor 3D point cloud map — Location 1" src="https://github.com/user-attachments/assets/aa9b1b53-225e-434d-acb7-33786b49a3f1" />
  <br><em>Outdoor 3D point cloud map — Location 1</em>
</p>

<p align="center">
  <img width="1150" alt="Outdoor 3D point cloud map — Location 2" src="https://github.com/user-attachments/assets/b8f0c1a0-3e87-4714-9ace-ba185fd7a1a9" />
  <br><em>Outdoor 3D point cloud map — Location 2</em>
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
- PX4

**Software**
- ROS 2 Jazzy
- RTAB-Map
- uXRCE-DDS

    </td>
    <td colspan="2">

**Hardware**
- Unitree L2 3D LiDAR
- NVIDIA Jetson Orin Nano
- Cube Orange+
- Holybro X500 V2
- PX4

**Software**
- ROS 2 Jazzy
- Point-LIO
- uXRCE-DDS

    </td>
  </tr>
</table>

---

## 🏗️ System Architecture

```text
                         GNSS-DENIED QUADCOPTER
                                  │
                    ┌─────────────┴─────────────┐
                    │                           │
                 D455F                     Unitree L2
              RGB + Depth                    + IMU
                    │                           │
                    ▼                           ▼
             RGB-D VO +                    Point-LIO
              RTAB-Map                        LIO
                    │                           │
                    └─────────────┬─────────────┘
                                  ▼
                         Jetson Orin Nano
                            ROS 2 Jazzy
                                  │
                             uXRCE-DDS
                                  │
                                  ▼
                         Cube Orange+ / PX4
                                  │
                                  ▼
                              Quadcopter
```

The VO and LIO pipelines are **independent alternatives**; they are not fused together in the current implementation.

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

#### ⚙️ Configuration

The main RTAB-Map configuration used for real-time testing includes:

```text
Max Features       : 150
Minimum Inliers    : 20
Feature Type       : 6
Odometry Strategy  : 0
F2M Maximum Size   : 500
NNDR               : 0.8
```

Bundle adjustment was disabled in the real-time configuration to reduce computational load.

#### 🔌 ROS 2 Inputs

```text
RGB:
 /camera/camera/color/image_raw

Depth:
 /camera/camera/aligned_depth_to_color/image_raw

Camera Info:
 /camera/camera/color/camera_info

Frame:
 camera_link
```

#### 🔧 D455F IMU

The D455F onboard IMU is **not used** in this implementation due to a JetPack/kernel compatibility issue on the Jetson.

Therefore, this pipeline is referred to as **RGB-D Visual Odometry (VO)** rather than Visual-Inertial Odometry (VIO).

---

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

Point-LIO provides real-time LiDAR–inertial state estimation while accumulating a 3D point-cloud map of the environment.

Unlike the RGB-D pipeline, LIO does not depend on image features and directly exploits LiDAR geometry together with inertial measurements.

---

## 🔗 PX4 Integration

Both localization pipelines run on the Jetson Orin Nano and communicate with the **Cube Orange+ running PX4** through **uXRCE-DDS**.

```text
Jetson Orin Nano
       │
   ROS 2 Jazzy
       │
   uXRCE-DDS
       │
       ▼
   Cube Orange+
       │
      PX4
       │
       ▼
 External Odometry
```

The estimated odometry from either VO or LIO is provided to the PX4 ROS 2 interface as external vehicle odometry and used by the PX4 estimator for navigation without GNSS position measurements.

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

## ⚠️ Limitations

- The D455F IMU is currently not used due to the JetPack/kernel compatibility issue.
- RGB-D VO depends on sufficient visual features and reliable depth measurements.
- LIO depends on LiDAR–IMU calibration, synchronization, and sufficient computational resources.
- Without GNSS, the system does not provide an absolute global position reference.
- Quantitative comparison against ground truth has not yet been performed.

---

## 🔮 Future Work

- Resolve D455F IMU compatibility and evaluate RGB-D VIO.
- Quantitatively evaluate VO and LIO against ground truth.
- Investigate visual–LiDAR fusion.
- Further optimize both pipelines for Jetson deployment.
- Evaluate autonomous waypoint navigation using external odometry.
