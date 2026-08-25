# GNSS_Denied_Quadcopter_Navigation


https://github.com/user-attachments/assets/e615a37a-06e0-4dc8-9170-f577e0f83028

# GNSS-Denied Quadcopter Navigation

A GNSS-denied navigation system for a quadcopter using two independent onboard perception approaches:

- **RGB-D Visual Odometry (VO) + RTAB-Map**
- **LiDAR–Inertial Odometry (LIO) using Point-LIO**

Both approaches run on an **NVIDIA Jetson Orin Nano** companion computer and provide external odometry to a **PX4** flight controller.

---

## System Overview

```text
                         GNSS-DENIED NAVIGATION
                                  │
                  ┌───────────────┴───────────────┐
                  │                               │
             RGB-D VO +                      LiDAR + IMU
             RTAB-Map                        Point-LIO
                  │                               │
                  └───────────────┬───────────────┘
                                  │
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

The two approaches are evaluated **independently** as alternative solutions for localization and navigation without GNSS.

---

## Hardware

| Component | Hardware |
|---|---|
| Airframe | Holybro X500 V2 |
| Flight Controller | Cube Orange+ |
| Autopilot | PX4 |
| Companion Computer | NVIDIA Jetson Orin Nano |
| VO Sensor | Intel RealSense D455F |
| LIO Sensor | Unitree L2 3D LiDAR |
| Motors | *To be added* |
| ESCs | *To be added* |

---

## Software

- Ubuntu / NVIDIA JetPack on Jetson Orin Nano
- ROS 2 Jazzy
- PX4 Autopilot
- RTAB-Map
- Point-LIO
- uXRCE-DDS

---

# 1. RGB-D Visual Odometry + RTAB-Map

The first approach uses an **Intel RealSense D455F** to provide RGB and depth data.

RTAB-Map is used for RGB-D visual odometry and SLAM. The estimated odometry is provided to PX4 as external visual odometry.

### Pipeline

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
       └── RTAB-Map SLAM
             │
             ├── Loop Closure
             └── Pose Graph Optimization
       │
       ▼
 External Odometry
       │
       ▼
      PX4
```

### Camera Topics

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

### RTAB-Map Configuration

The main visual-odometry configuration used during testing includes:

```text
Max Features       : 150
Minimum Inliers    : 20
Feature Type       : 6
Odometry Strategy  : 0
F2M Maximum Size   : 500
NNDR               : 0.8
```

Bundle adjustment was disabled in the real-time odometry configuration to reduce computational load.

### D455F IMU

The D455F's onboard IMU was not used in this implementation due to a **JetPack/kernel compatibility issue on the Jetson**.

Therefore, this branch is referred to as **RGB-D Visual Odometry (VO)** rather than Visual-Inertial Odometry (VIO).

---

# 2. LiDAR–Inertial Odometry — Point-LIO

The second approach uses a **Unitree L2 3D LiDAR** together with its inertial measurements.

Point-LIO performs LiDAR–inertial state estimation and generates a real-time 3D point-cloud map.

### Pipeline

```text
Unitree L2
 │
 ├── 3D LiDAR
 └── IMU
       │
       ▼
    Point-LIO
       │
       ├── LiDAR-Inertial Odometry
       └── 3D Point Cloud Map
       │
       ▼
 External Odometry
       │
       ▼
      PX4
```

LIO provides geometric localization without relying on image features or GNSS.

---

# 3. PX4 Integration

Both navigation pipelines run on the Jetson Orin Nano and communicate with the **Cube Orange+ running PX4** through **uXRCE-DDS**.

```text
Jetson Orin Nano
       │
     ROS 2
       │
   uXRCE-DDS
       │
       ▼
   Cube Orange+
       │
      PX4
```

The estimated external odometry is published to the PX4 ROS 2 interface and consumed by the PX4 estimator for navigation.

> Verify the exact `/fmu/in/...` topic used by your current PX4 configuration before deployment, as the topic depends on the PX4 ROS 2 interface/version.

---

# 4. GNSS-Denied Navigation

The system is designed to operate without GNSS position measurements.

The navigation estimate is instead generated from onboard perception:

```text
       GNSS DENIED
            │
     ┌──────┴──────┐
     │             │
    RGB-D        LiDAR + IMU
     │             │
   RTAB-Map      Point-LIO
     │             │
     ▼             ▼
   VO Pose       LIO Pose
     │             │
     └──────┬──────┘
            ▼
           PX4
```

This allows the same quadcopter platform to be evaluated using two different sensor-based localization approaches.

---

# 5. Demonstration

The project includes separate demonstrations for both approaches.

### RGB-D VO + RTAB-Map

Demonstrates:

- GNSS-denied operation
- Real-time RGB-D visual odometry
- RTAB-Map SLAM
- Loop-closure-based map optimization
- Estimated trajectory
- Reconstructed environment

### Point-LIO

Demonstrates:

- GNSS-denied operation
- Real-time LiDAR–inertial odometry
- Estimated trajectory
- 3D point-cloud reconstruction
- External odometry integration with PX4

Demo videos are available in [`media/`](media/).

---

# 6. VO vs LIO

| | RGB-D VO + RTAB-Map | Point-LIO |
|---|---|---|
| Primary sensor | D455F RGB-D | Unitree L2 3D LiDAR |
| IMU | Not used | Used |
| GNSS | Not required | Not required |
| Odometry | RGB-D visual | LiDAR–inertial |
| Mapping | RGB-D / RTAB-Map | 3D point cloud |
| Loop closure | RTAB-Map | Point-LIO odometry/map |
| Main dependency | Visual features + depth | LiDAR geometry + IMU |
| Lighting sensitivity | Higher | Lower |
| 3D geometric information | Depth-based | Direct LiDAR |

The approaches provide different trade-offs rather than one being universally superior.

---

# 7. Repository Structure

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

# 8. Limitations

- D455F IMU integration was not used due to the JetPack/kernel compatibility issue.
- RGB-D VO depends on sufficient visual features and reliable depth measurements.
- LIO requires greater computational resources and depends on LiDAR–IMU calibration and synchronization.
- Without GNSS, the system does not provide an absolute global position reference.
- Final navigation accuracy depends on sensor calibration, environment, estimator configuration, and flight dynamics.

---

# 9. Future Work

- Resolve D455F IMU compatibility and evaluate true RGB-D VIO.
- Quantitatively compare VO and LIO against ground truth.
- Investigate visual–LiDAR fusion for improved robustness.
- Optimize both pipelines for real-time Jetson deployment.
- Evaluate performance across different lighting and environmental conditions.
- Improve autonomous waypoint navigation using external odometry.

---

## Acknowledgements

This project uses:

- [ROS 2](https://docs.ros.org/)
- [PX4 Autopilot](https://px4.io/)
- [RTAB-Map](https://introlab.github.io/rtabmap/)
- [Point-LIO](https://github.com/hku-mars/Point-LIO)
- [Intel RealSense ROS](https://github.com/IntelRealSense/realsense-ros)
