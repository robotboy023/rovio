# Running ROVIO on a Custom Camera–IMU Sensor Setup

This document describes the requirements and recommended practices for running **ROVIO (Robust Visual–Inertial Odometry)** on a custom hardware platform.

ROVIO is a tightly coupled VIO system and is sensitive to **timestamping**, **calibration**, and **configuration quality**. Following the guidelines below is critical for stable and accurate performance.

---

## 1. Hardware Requirements

ROVIO is primarily designed to operate with a **camera + IMU** setup.

### Required Sensors
- **IMU** (required)  
  Any IMU grade can be used. Consumer-grade IMUs can achieve good performance **if properly calibrated** and if time synchronization is handled correctly.

- **Camera** (required)
    - **Global shutter cameras** are strongly recommended.
    - **Rolling shutter cameras** can be used, but may:
        - Degrade accuracy
        - Increase estimator instability
        - Cause filter divergence in aggressive motion

### Optional Sensors
- **GPS**
- **Optical flow / velocity sensors**

---

## 2. Timestamping and Clock Expectations

ROVIO assumes **all sensor streams are timestamped using a consistent time base**.

### Best Practices
- Timestamp data **as close to the sensor driver as possible**
- Avoid timestamping at higher-level processing nodes
- If sensors come from different hardware clocks:
    - Ensure a hardware or software synchronization mechanism
    - Or maintain accurate, static time-offset estimates

---

## 3. Calibration Overview

Calibration consists of two parts:

| Type | Description |
|---|---|
| **Intrinsics** | Sensor-specific internal parameters |
| **Extrinsics** | Relative transforms between sensors |

---

## 4. Intrinsics Calibration

### IMU Intrinsics
IMU calibration parameters typically include:
- Noise density
- Bias random walk

Recommended tools:
- https://github.com/ori-drs/allan_variance_ros
- https://github.com/rpng/kalibr_allan

**Configuration file:**  
`rovio/cfg/rovio.info` (lines ~152–161)

---

### Camera Intrinsics

Supported camera models:
- **Radtan (plumb_bob)** – standard lenses (<90° FOV)
- **Equidistant (fisheye)** – wide-angle lenses
- **Double Sphere** – ultra-wide lenses (>180° FOV)

Recommended calibration tools:
- https://docs.nav2.org/tutorials/docs/camera_calibration.html
- https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

**Configuration file:**  
`rovio/cfg/euroc_cam0.yaml`

---

## 5. Extrinsics Calibration

### IMU ↔ Camera
Extrinsics define:
- Relative pose between IMU and camera
- Static time offset between sensors

Recommended tool:
- https://github.com/ethz-asl/kalibr

ROVIO expects:
- Quaternion: rotation from **camera → IMU**
- Translation: from **IMU → camera**

**Configuration file:**  
`rovio/cfg/rovio.info` (lines ~11–18)

---

## 6. Topic Configuration

Edit:
```
rovio/launch/ros2_rovio_node_launch.yaml
```
or
```
rovio/launch/ros2_rovio_rosbag_loader_launch.yaml
```

Update:
- `imu_topic`
- `cam0_topic`

---

## 7. Computation and Performance Tuning

Recommended for low-compute platforms:
- Reduce image resolution. Recommended: **320x240**. Modify in `ros2_rovio_node_launch.yaml` and `ros2_rovio_rosbag_loader_launch.yaml.`
- Number of features: **10–15**. Modify in `CMakeLists.txt` in the rovio folder.
- Patch size: **4**. Modify in `CMakeLists.txt` in the rovio folder.
- `maxIterations` **~10**. Modify in `rovio.info` file.
- `alignMaxUniSample 1`. Modify in `rovio.info` file.
- `useFeatureDetectorScoreSelection true`. Modify in `rovio.info` file. Uses Fast Score for feature selection instead of shi-tomasi score which is computationally expensive. Might notice some degradation in performance and improvement in computational efficiency, less computational peaks.
---

## 8. Modifying Compilation Optimizations

Compilation optimizations are controlled in `CMakeLists.txt` under the compiler settings block.
They are recommended settings for Raspberry Pi 5/4 and Raspberry Pi 3 respectively.
### Current platform-specific flags
- **ARM 64-bit (`arm64|aarch64`)**
  - `-march=armv8-a -mtune=cortex-a76 -Ofast -floop-block -floop-interchange -fno-rounding-math -fprefetch-loop-arrays`
- **ARM 32-bit (`arm`)**
  - `-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -Ofast -floop-block -floop-interchange -fno-rounding-math -fprefetch-loop-arrays`
- **Non-ARM**
  - `-march=native -Ofast -floop-block -floop-interchange -fno-rounding-math -fprefetch-loop-arrays`

Update these flags in `CMakeLists.txt` to match your CPU or toolchain requirements.
Visit this [url](https://gist.github.com/fm4dd/c663217935dc17f0fc73c9c81b0aa845)
for additional compilation flags for a custom board.
---

## 9. Final Notes

- ROVIO is sensitive to timestamping and calibration quality
- Validate with recorded datasets before deploying on hardware
