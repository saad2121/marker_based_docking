# ROS Dual Reflector Docking

## Overview

This package implements a **LiDAR-based autonomous docking system** for mobile robots using **two retro-reflective markers** with known geometric separation.

The system detects a **pair of reflectors** from `LaserScan` data, computes the **geometric center point** between them, and guides the robot to dock using a **state-aware closed-loop controller**.

### Key Features
- Uses **2D LiDAR only** (no camera, no fiducials)
- Robust against noise using **clustering and geometric validation**
- **Predock dwell gating** for AMCL-safe operation
- Fully **parameterized using YAML**
- Publishes **docking state, distance, and angle** for higher-level integration

---

## Package Contents

### Nodes

| Node | Description |
|----|----|
| `dual_reflector_detector` | Detects reflector pairs from LiDAR data |
| `dual_reflector_docking_controller` | Executes docking control and state management |

### Messages

| Message | Description |
|----|----|
| `DockTarget.msg` | Distance and angle to docking target |
| `DockingStatus.msg` | Docking state, distance, and angle |

---

## System Architecture

<img width="2475" height="1163" alt="image" src="https://github.com/user-attachments/assets/94d3980a-17c7-4533-a7d2-25d0dcffff56" />

## Code Flow

### 1. Reflector Detection
- LaserScan data is segmented into contiguous clusters
- Clusters are filtered based on:
  - Number of points
  - Maximum range
- All cluster pairs are evaluated using known reflector separation
- The midpoint of a valid reflector pair becomes the docking target

### 2. Predock Validation
- Robot must reach a predefined **predock pose**
- Pose must remain stable for a configurable **dwell time**
- Docking is armed only after dwell condition is satisfied

### 3. Docking Control
- Angular error is smoothed using a **moving average**
- PD control aligns the robot with the reflector centerline
- Linear motion is enabled only when angular error is small
- Docking completes when distance and angle are within tolerance

---

## Launch File

### `dual_reflector_dock.launch`

This launch file:
- Starts the simulation environment
- Launches the dual reflector detector
- Launches the docking controller
- Starts the navigation stack

```bash
roslaunch docking_marker dual_reflector_dock.launch
```


## Node Interfaces
### Dual Reflector Detector

|   Topic | Type                    | Description    | Published/Subscribed |
| ------: | ----------------------- | -------------- | -------------------- |
| `/scan` | `sensor_msgs/LaserScan` | Raw LiDAR scan | Subscribed           |
| `/dock_target` | `DockTarget` | Docking target relative to robot | Published|

### Docking Controller

|   Topic | Type                    | Description    | Published/Subscribed |
| ------: | ----------------------- | -------------- | -------------------- |
| `/amcl_pose` | `PoseWithCovarianceStamped` | Robot localization | Subscribed |
| `/dock_target` | `DockTarget` | Distance and angle to target | Subscribed |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands | Published|
| `/docking_status` | `DockingStatus` | Docking state and feedback | Published|


Published using `DockingStatus.msg`

```
uint8 state
float32 distance
float32 angle
```

State Encoding

| Value | State   |
| ----- | ------- |
| 0     | IDLE    |
| 1     | PREDOCK |
| 2     | DOCKING |
| 3     | DOCKED  |

---

## Mathematical Modeling 

This section describes the mathematical formulation used for **dual reflector detection**, **target center computation**, and **control error calculation** for LiDAR-based docking.

### 1. Reflector Cluster Representation

Each detected reflector cluster from the LiDAR scan is represented in polar coordinates relative to the robot frame:

$$
(r_i,\; \theta_i)
$$

Where:
- $r_i$ is the average range of the cluster
- $\theta_i$ is the angular position of the cluster

These polar coordinates are converted into Cartesian coordinates:

$$
x_i = r_i \cos(\theta_i)
$$

$$
y_i = r_i \sin(\theta_i)
$$

---

### 2. Dual Reflector Pair Validation

Given two reflector clusters $i$ and $j$, their Euclidean separation is computed as:

$$
d_{ij} = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2}
$$

The reflector pair is considered **valid** if the measured separation matches the known physical separation:

$$
\left| d_{ij} - D_{\text{ref}} \right| < \varepsilon
$$

Where:
- $D_{\text{ref}}$ is the known reflector separation (e.g., 0.238 m)
- $\varepsilon$ is the allowable separation tolerance

This constraint ensures that **only the true reflector pair** is selected and false positives are rejected.

---

### 3. Docking Target Center Point Calculation

Once a valid reflector pair is identified, the docking target is defined as the **midpoint** between the two reflectors:

$$
x_c = \frac{x_i + x_j}{2}
$$

$$
y_c = \frac{y_i + y_j}{2}
$$

The midpoint represents the **centerline of the docking station**.

The target position is then converted back to polar coordinates relative to the robot:

### Distance to Docking Target

$$
d = \sqrt{x_c^2 + y_c^2}
$$

### Angular Error to Docking Target

$$
\alpha = \tan^{-1}\left(\frac{y_c}{x_c}\right)
$$

Where:
- $d$ is the distance error
- $\alpha$ is the heading (angular) error

---

### 4. Angle Filtering (Moving Average)

To reduce measurement noise and avoid discontinuities near $\pm \pi$, angular measurements are filtered using a **circular moving average**:

$$
\bar{\alpha} =
\tan^{-1}
\left(
\frac{\sum_{k=1}^{N} \sin(\alpha_k)}
{\sum_{k=1}^{N} \cos(\alpha_k)}
\right)
$$

Where:
- $\alpha_k$ are recent angle measurements
- $N$ is the size of the averaging window

This formulation correctly handles angle wrap-around.

---

### 5. Control Error Definitions

### Distance Error

The distance error used by the controller is defined as:

$$
e_d = d - d_{\text{offset}}
$$

Where:
- $d_{\text{offset}}$ is the desired final docking distance

---

### Angular Error

The angular control error is:

$$
e_\theta = \bar{\alpha}
$$

---

### 6. Control Laws

### Angular Velocity Control (PD)

The angular velocity command is computed as:

$$
\omega = K_p^{\theta} \ e_\theta + K_d^{\theta} \ \frac{d e_\theta}{dt}
$$

---

### Linear Velocity Control (PD)

Linear motion is enabled only when angular alignment is acceptable:

$$
|e_\theta| < \theta_{\text{threshold}}
$$

The linear velocity command is then computed as:

$$
v = K_p^{d} \ e_d + K_d^{d} \ \frac{d e_d}{dt}
$$

---

### 7. Docking Completion Condition

Docking is considered complete when both distance and angular errors satisfy:

$$
|e_d| < d_{\text{tol}}
$$

$$
|e_\theta| < \theta_{\text{tol}}
$$

Where:
- $d_{\text{tol}}$ is the distance tolerance
- $\theta_{\text{tol}}$ is the angular tolerance

---

## Summary

The dual reflector docking approach relies on:
- **Geometric validation** of reflector pairs
- **Midpoint computation** for accurate docking alignment
- **Noise-robust angular filtering**
- **Decoupled PD control** for angular and linear motion

This formulation ensures stable, accurate, and repeatable docking using only 2D LiDAR data.
