SlimeVR ArUco Optical Correction Tracker

A standalone optical correction system for SlimeVR trackers using ArUco markers. This script captures a webcam feed, calculates the absolute spatial orientation of a fiducial marker, and sends rotational correction data to the SlimeVR server via OSC.

This project is designed to minimize the primary issue with DIY IMU-based trackers (BMI160, MPU6050, etc.)—Yaw drift—providing stable tracking without the need for constant manual resets.

🌟 Features

High-Precision IPPE_SQUARE Solver: Ensures stable pose estimation for planar ArUco markers.

Quaternion Math Stabilization: Implements hemisphere alignment and angular velocity extrapolation to completely eliminate 180° axis "flips" (pose ambiguity) and smooth out micro-jitter.

Built-in Camera Calibration: An interactive calibration tool using a ChArUco 5x7 board to calculate and save the camera matrix and lens distortion coefficients.

Dual Operation Modes: * Preview Mode: A 3D UI mode for debugging, testing distances, and checking lighting.

Headless Mode: Pure background tracking without GUI rendering to save CPU resources.

Protective Filtering: Filters markers based on minimum perimeter (distance), steep viewing angles, and sudden rotational jumps (glitch detection).

🛠️ Installation & Setup

1. Requirements

Python 3.10 or 3.11

opencv-contrib-python (The standard opencv-python package does not include the aruco module).

2. Environment Setup

Open your terminal in the project directory and run:

# Initialize a virtual environment
python -m venv venv

# Activate the environment (Windows)
.\venv\Scripts\activate
# Activate the environment (Linux/MacOS)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt




3. Run the Tracker

python main.py




🚀 Quick Start Guide

Follow these steps to configure the tracker for the first time:

1. Camera Index: Find your camera by selecting the correct number in the Camera Index field (0 is usually the default webcam, 1 for the secondary, etc.).

2. Marker Size: Enter the physical size of your printed ArUco marker in the Marker Size (mm) field.

How to measure: Measure the exact length of the outer black square border, not the inner white pattern.

3. Marker IDs: Enter the IDs of the markers you are using in the Marker IDs field, separated by commas (e.g., 1, 2, 3).

4. Camera Calibration: * Print the charuco_5x7_calibration.pdf file at exactly 100% scale.

Click the 📷 CALIBRATE button.

Take at least 10 photos of the board in different parts of the frame by pressing the C key.

Press ENTER to save the results.

5. Tuning Filters: * Click 👁 Preview.

Move the marker around to find the extreme angles where tracking starts to degrade, and input these limits into Min Angle and Max Angle.

Step away from the camera to find the distance where the marker becomes unstable, and enter the perimeter size shown on the screen into Min Perim.

6. Start Tracking: Click ▶ START to begin the headless tracking process.

7. SlimeVR Sync: In SlimeVR, perform a Full Reset while showing the active markers to the camera to align the tracking spaces.

⚙️ Additional Features

REC LOG (Telemetry Recording): Click this button to record telemetry data (quaternions, tilt angles, and deltas) into a CSV file. This is extremely useful for algorithm stability analysis and debugging tracking jumps.

OSC IP & Port: By default, data is sent to the local SlimeVR server (127.0.0.1:9005). You can change these network settings to route the tracking data to other applications (e.g., VRChat, Unity, or custom Python scripts) for other specific tasks.

Max Jump (deg): A built-in glitch filter. It sets the maximum allowed angle delta between consecutive frames. If the marker orientation jumps more than this value, the frame is ignored.

🔗 SlimeVR Integration (WIP)

Note: This tracker script is part of a larger full-stack integration. It requires corresponding changes in the SlimeVR Server and SolarXR Protocol to function completely.

Protocol Data (OSC)

The script broadcasts OSC packets to the /aruco/correction address with the following argument structure:
[int: marker_id, float: qx, float: qy, float: qz, float: qw, float: timestamp_ms, int: is_stable]

Server/GUI Mod Features:

When used with the modified SlimeVR server branch, you get access to:

Marker ID Assignment: Assign a specific ArUco marker ID to each individual tracker directly in the SlimeVR GUI.

Camera Reset: A dedicated button to quickly reset the optical correction applied to the tracker.
