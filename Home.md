# SlimeVR ArUco Optical Correction Tracker

A standalone optical correction system for SlimeVR trackers using ArUco markers. This script captures a webcam feed, calculates the absolute spatial orientation of a fiducial marker, and sends rotational correction data to the SlimeVR server via OSC.

This project is designed to minimize the primary issue with DIY IMU-based trackers (BMI160, MPU6050, etc.)—Yaw drift—providing stable tracking without the need for constant manual resets.

## 🌟 Features

- **High-Precision IPPE_SQUARE Solver**: Ensures stable pose estimation for planar ArUco markers.

- **Quaternion Math Stabilization**: Implements hemisphere alignment and angular velocity extrapolation to completely eliminate 180° axis "flips" (pose ambiguity) and smooth out micro-jitter.

- **Built-in Camera Calibration**: An interactive calibration tool using a ChArUco 5x7 board to calculate and save the camera matrix and lens distortion coefficients.

- **Dual Operation Modes**:
  - *Preview Mode*: A 3D UI mode for debugging, testing distances, and checking lighting.
  - *Headless Mode*: Pure background tracking without GUI rendering to save CPU resources.

- **Protective Filtering**: Filters markers based on minimum perimeter (distance), steep viewing angles, and sudden rotational jumps (glitch detection).

## 🛠️ Installation & Setup

### Requirements

- Python 3.10 or 3.11
- opencv-contrib-python (The standard opencv-python package does not include the aruco module).

### Environment Setup

Open your terminal in the project directory and run:

```bash
# Initialize a virtual environment
python -m venv venv

# Activate the environment (Windows)
.\venv\Scripts\activate

# Activate the environment (Linux/MacOS)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Run the Tracker

```bash
python main.py
```

## 🚀 Quick Start Guide

Follow these steps to configure the tracker for the first time:

### 1. Camera Index

Find your camera by selecting the correct number in the Camera Index field (0 is usually the default webcam, 1 for the secondary, etc.).

![Camera Index](https://github.com/user-attachments/assets/864f6b38-be1a-4bcf-9136-cea56809789f)

### 2. Marker Size

Enter the physical size of your printed ArUco marker in the Marker Size (mm) field.

![Marker Size](https://github.com/user-attachments/assets/81aa4c2c-a20a-411b-813e-e6eda86be948)

**How to measure**: Measure the exact length of the outer black square border, not the inner white pattern.

![Measurement](https://github.com/user-attachments/assets/0c127d5d-0d0a-4d36-b491-85bceff1529f)

### 3. Marker IDs

Enter the IDs of the markers you are using in the Marker IDs field, separated by commas (e.g., 1, 2, 3).

![Marker IDs](https://github.com/user-attachments/assets/b16e2d9e-40b2-42bd-9c9d-608ce518325d)

### 4. Camera Calibration

1. Print the charuco_5x7_calibration.pdf file at exactly 100% scale.
2. Click the 📷 **CALIBRATE** button.
3. Take at least 10 photos of the board in different parts of the frame by pressing the **C** key.
4. Press **ENTER** to save the results.

![Calibration](https://github.com/user-attachments/assets/274366b7-1b9a-4b59-9f59-0a977a80c889)

### 5. Tuning Filters

**Angle limits:**
- Click 👁 **Preview**.
- Move the marker around to find the extreme angles where tracking starts to degrade, and input these limits into Min Angle and Max Angle.

![Angle Tuning](https://github.com/user-attachments/assets/458d1dbc-c59c-4ce2-8d21-95919f79efcc)

**Distance limits:**
- Step away from the camera to find the distance where the marker becomes unstable, and enter the perimeter size shown on the screen into Min Perim.

![Distance Tuning](https://github.com/user-attachments/assets/b9f662dd-5344-4365-8818-a2a8dede0278)

### 6. Start Tracking

Click ▶ **START** to begin the headless tracking process.

### 7. SlimeVR Sync

In SlimeVR, perform a Full Reset while showing the active markers to the camera to align the tracking spaces.

## ⚙️ Additional Features

### REC LOG (Telemetry Recording)

Click this button to record telemetry data (quaternions, tilt angles, and deltas) into a CSV file. This is extremely useful for algorithm stability analysis and debugging tracking jumps.

### OSC IP & Port

By default, data is sent to the local SlimeVR server (127.0.0.1:9005). You can change these network settings to route the tracking data to other applications (e.g., VRChat, Unity, or custom Python scripts) for other specific tasks.

### Max Jump (deg)

A built-in glitch filter. It sets the maximum allowed angle delta between consecutive frames. If the marker orientation jumps more than this value, the frame is ignored.

## 🔗 SlimeVR Integration (WIP)

Note: This tracker script is part of a larger full-stack integration. It requires corresponding changes in the SlimeVR Server and SolarXR Protocol to function completely.

### Protocol Data (OSC)

The script broadcasts OSC packets to the `/aruco/correction` address with the following argument structure:

```
[int: marker_id, float: qx, float: qy, float: qz, float: qw, float: timestamp_ms, int: is_stable]
```

### Server/GUI Mod Features

When used with the modified SlimeVR server branch, you get access to:

- **Marker ID Assignment**: Assign a specific ArUco marker ID to each individual tracker directly in the SlimeVR GUI.
- **Camera Reset**: A dedicated button to quickly reset the optical correction applied to the tracker.
