# ArUco Integration in SlimeVR Server

This page describes the architecture and features of ArUco optical correction integration in SlimeVR Server written in Kotlin.

## 📋 Architecture Overview

The ArUco integration consists of two main components:

1. **ArucoOscReceiver** — network receiver for data from the optical tracker
2. **ArucoProcessor** — correction handler with PID controller and IMU history buffering

These components work asynchronously and thread-safely thanks to `ConcurrentHashMap`.

## 🔌 ArucoOscReceiver — OSC Data Receiver

### Purpose

Listens for incoming OSC packets at the `/aruco/correction` address from the optical tracker and stores the latest data for each marker.

### Data Structure

```kotlin
data class ArucoData(
    val qx: Float,
    val qy: Float,
    val qz: Float,
    val qw: Float,
    val timestampMs: Long,
    val isStable: Boolean,
)
```

**Fields:**
- `qx, qy, qz, qw` — quaternion components representing marker orientation in camera space
- `timestampMs` — frame capture timestamp (milliseconds)
- `isStable` — stability flag from the optical tracker

### OSC Packet Format

```
Address: /aruco/correction
Arguments: [int: marker_id, float: qx, float: qy, float: qz, float: qw, float: timestamp_ms, int: is_stable]
```

### Implementation Features

- **Thread-Safety**: Uses `ConcurrentHashMap` to store the latest data for each marker
- **No Queue**: Stores only the **latest** packet for each marker, old data is overwritten
- **Error Protection**: Try-catch block during parsing prevents application crash from malformed packets
- **Minimal Latency**: No message queue buffering — all data is fresh

## 🎯 ArucoProcessor — Main Correction Handler

### Architecture

```
Incoming ArUco data
        ↓
[Coordinate system transformation]
        ↓
[Yaw extraction from quaternions]
        ↓
[1D calibration (flat plane)]
        ↓
[Drift calculation]
        ↓
[PID controller]
        ↓
[Correction offset generation]
        ↓
Application to tracker in adjustToReference()
```

### Key Components

#### 1. RingBuffer — IMU History

Circular buffer stores history of orientations and offsets with timestamps:

```kotlin
private class RingBuffer(val capacity: Int) {
    private val times = LongArray(capacity)
    private val rotations = Array(capacity) { Quaternion.IDENTITY }
    private val offsets = Array(capacity) { Quaternion.IDENTITY }
    private var head = 0
    
    fun add(time: Long, rot: Quaternion, offset: Quaternion)
    fun getClosestFrame(targetTime: Long): HistoryFrame?
}
```

**Purpose:**
- Synchronizes timestamps between ArUco (camera) and IMU (tracker)
- When an ArUco packet arrives with timestamp `t`, finds the nearest IMU frame from history
- Solves the **asynchronous devices** problem

**Parameters:**
- `capacity = 1000` — Server tickrate is 1000 Hz, buffer holds 1 second of history
- At server's 1000 Hz update frequency, this provides enough lookback to match ArUco data with historical IMU state

#### 2. extractYaw() — Yaw Component Isolation

```kotlin
private fun extractYaw(q: Quaternion): Quaternion {
    val mag = sqrt(q.w * q.w + q.y * q.y)
    return if (mag > 0.0001f) {
        var nw = q.w / mag
        var ny = q.y / mag
        if (nw < 0) {
            nw = -nw
            ny = -ny
        }
        Quaternion(nw, 0f, ny, 0f)
    } else {
        Quaternion.IDENTITY
    }
}
```

**What it does:**
- Removes Roll (X) and Pitch (Z) from the quaternion
- Leaves only **pure Yaw** — the compass
- Normalizes w and y components, zeroing x and z

**Why it's needed:**
- Camera can be tilted or elevated
- Camera tilt → Pitch/Roll error from ArUco
- This error shouldn't affect Yaw correction
- **Solution**: Extract only the compass component

#### 3. Calibration (Flat Plane)

```kotlin
if (!yawCalibrations.containsKey(markerId)) {
    yawCalibrations[markerId] = currentImuYaw * currentArucoYaw.inv()
    lastYawError[markerId] = 0f
    println("[ArUco Processor] [Marker $markerId] 1D YAW CALIBRATION SET!")
    continue
}
```

**What happens:**
- First time camera sees marker → saves **difference** between "North" of IMU and "North" of ArUco
- This difference = `currentImuYaw * currentArucoYaw.inv()`
- All subsequent corrections are applied **relative to this calibration**

**Feature:**
- **1D calibration** — only once, only for Yaw
- No calibration drift over time
- Use the **Optical Reset** button to recalibrate after tracker reboot

#### 4. Drift Calculation

```kotlin
val offsetToRoom = yawCalibrations[markerId]!!
val targetImuYaw = offsetToRoom * currentArucoYaw

var driftErrorYaw = targetImuYaw * currentImuYaw.inv()
val errorAngleDeg = 2.0 * acos(driftErrorYaw.w) * (180.0 / PI)
```

**Logic:**
1. Take what **should be** (targetImuYaw) — ArUco translated to IMU space
2. Take what **actually is** (currentImuYaw) — raw IMU data
3. Find the difference (drift) = `target * actual.inv()`
4. Convert error quaternion to **angle in degrees**

#### 5. Glitch Detection (Outlier Protection)

```kotlin
if (errorAngleDeg > 45.0) {
    println("[ArUco Processor] [Marker $markerId] WARNING: Camera jump. Recalibrating...")
    yawCalibrations.remove(markerId)
    continue
}
```

**What it prevents:**
- If error > 45° — this is clearly not drift, but a camera failure
- **Marker leaves frame** → ArUco returns garbage
- **Solution**: Reset calibration for this specific marker only
- Next frame will start fresh calibration

#### 6. PID Controller

```kotlin
val kp = 0.05f   // Proportional coefficient
val ki = 0.001f  // Integral coefficient

val prevIntegral = lastYawError.getOrDefault(markerId, 0f)
val newIntegral = prevIntegral + signedErrorRadian
val limitedIntegral = newIntegral.coerceIn(-0.5f, 0.5f)
lastYawError[markerId] = limitedIntegral

val pidCorrection = (kp * signedErrorRadian + ki * limitedIntegral)
    .coerceIn((-PI / 4).toFloat(), (PI / 4).toFloat())
```

**How it works:**

| Component | Coefficient | Purpose |
|-----------|-----------|---------|
| **P** (Proportional) | 0.05 | Immediate reaction to error |
| **I** (Integral) | 0.001 | Error accumulation for slow corrections |
| **D** (Derivative) | ❌ | Not used (not needed) |

**Features:**
- PID output is limited: `[-45°, +45°]` (can't jump further)
- Integral is limited: `[-0.5, +0.5]` rad (anti-windup)
- Correction is applied **only to Yaw**, doesn't touch Pitch/Roll

#### 7. Offset Generation

```kotlin
val correctionHalfAngle = pidCorrection / 2.0f
val correctedW = cos(correctionHalfAngle)
val correctedY = sin(correctionHalfAngle)

val yawCorrectionStep = Quaternion(correctedW, 0f, correctedY, 0f)
val newYawOffset = yawCorrectionStep * activeYawOffset
currentArucoOffsets[markerId] = newYawOffset
```

**What happens:**
1. PID outputs error in radians
2. Convert to **half angle** (for quaternion representation)
3. Create pure Yaw quaternion: `Quaternion(cos(θ/2), 0, sin(θ/2), 0)`
4. **Add** to current offset (don't replace, accumulate)
5. Save new offset

## 🔄 Integration in adjustToReference()

### Integration Point in Transformation Chain

```kotlin
fun adjustToReference(rotation: Quaternion): Quaternion {
    var rot = rotation
    
    // Other transformations...
    rot *= mountingOrientation
    rot = gyroFix * rot
    rot *= attachmentFix
    rot = mountRotFix.inv() * (rot * mountRotFix)
    rot *= tposeDownFix

    // =========================================================================
    // ✨ ARUCO INTEGRATION (GRAVITY-ALIGNED SPACE)
    // =========================================================================
    lastGravityAligned = rot  // ← Save "clean" pose before optical correction
    rot = opticalDriftFix * rot  // ← Apply ArUco offset
    lastOpticallyCorrected = rot
    // =========================================================================

    // Yaw Reset from GUI (already contains ArUco correction)
    rot = yawFix * rot
    rot = constraintFix * rot

    return rot
}
```

### Why Here?

```
Transformation chain:
[Raw IMU] → [Mounting Fix] → [Gyro Fix] → [Attachment Fix] → [Mount Rot Fix] → [T-Pose Fix]
                                                                                        ↓
                                                            [⭐ ARUCO CORRECTION ⭐] ← Here!
                                                                                        ↓
                                                          [Yaw Fix] → [Constraint Fix]
```

**Conditions:**
- **Before Yaw Fix**: Offset not distorted by global heading reset yet
- **After other fixes**: Already gravity-aligned, ready for correction
- **In gravity-aligned space**: Yaw errors are pure, no Pitch/Roll mix

## 📊 Parameters and Their Impact

### PID Coefficients

```kotlin
val kp = 0.05f  // Proportional
val ki = 0.001f // Integral
```

**What to adjust:**
- ↑ `kp` → reacts faster to errors, but may oscillate
- ↓ `kp` → smoother, but corrects large errors slower
- ↑ `ki` → catches micro-drifts better, but may accumulate error
- ↓ `ki` → more stable, but may lag on linear drift

### RingBuffer Capacity

```kotlin
private val imuBuffers = ConcurrentHashMap<Int, RingBuffer>()
// in RingBuffer: capacity = 1000
```

With server running at 1000 Hz tickrate, the 1000-frame buffer = **1 second of history**. This is sufficient for optical trackers with typical network latency and frame processing delays.

### Glitch Detection Threshold

```kotlin
if (errorAngleDeg > 45.0) {
    yawCalibrations.remove(markerId)
    continue
}
```

45° is the threshold value. If drift is larger — it's a failure, not natural drift.

## 🖥️ GUI Integration

### Marker ID Configuration

An input field has been added to the tracker settings in the SlimeVR GUI:

- **Label**: "Aruco Marker ID"
- **Type**: Integer input field
- **Range**: 0-255 (standard ArUco marker ID range)
- **Effect**: Immediately associates the tracker with the specified optical marker
- **Persistence**: Saved with tracker configuration

### Optical Reset Button

A dedicated button provides manual recalibration:

- **Label**: "Optical Reset"
- **Function**: Clears all accumulated offsets and calibrations
- **Effect**: Resets `currentArucoOffsets` and `yawCalibrations` to empty state
- **Next step**: Show the marker to the camera again to re-trigger calibration
- **Use case**: After tracker restart, camera repositioning, or when experiencing persistent drift

**Code call:**
```kotlin
ArucoProcessor.requestOpticReset()
```

## 💾 Logging

### CSV Export

For each update, logs:

```
SysTime,MarkerID,ErrorDegrees,PID_Correction,Offset_W,Offset_Y
1718281200000,1,2.34,0.0012,0.9999,0.0456
1718281200050,1,2.10,0.0010,0.9999,0.0412
...
```

**File**: `aruco_debug_${currentTimeMillis}.csv`

**Usage:**
- Analyze correction stability
- Debug PID coefficients
- Detect error patterns

### Console Logs

Every 500 ms outputs status:

```
[ArUco Processor] M1 | Error: 2.3° | PID: 0.35° | Offset: 12°
```

## ⚙️ Features and Advantages

### ✅ Asynchronous Processing
- ArUco and IMU work at different frequencies
- RingBuffer synchronizes by timestamps
- No waiting between devices

### ✅ Thread-Safety
- `ConcurrentHashMap` for parallel access
- Multiple trackers can be processed simultaneously
- No race conditions

### ✅ Yaw Isolation
- extractYaw() extracts compass component only
- Camera tilt/elevation don't affect correction
- Robust to uneven camera mounting

### ✅ 1D Calibration
- Once at startup or after Optical Reset
- No calibration drift
- Simple one-click recalibration

### ✅ Outlier Protection
- Detects marker loss (>45°)
- Auto-recalibration of problematic marker only
- Other markers continue working

### ✅ PID Smoothing
- Smooth correction instead of jumps
- Anti-windup integral
- Maximum jump limit (±45°)

### ✅ Multiple Marker Support
- Independent offsets for each marker
- Can use multiple trackers with one camera
- Scales linearly with marker count

## 🔗 Related Components

- **Optical tracker** → Sends OSC packets to `/aruco/correction`
- **Tracker class** → Contains `arucoMarkerId` field and position fields
- **ResetsHandler** → Contains `opticalDriftFix` for applying correction
- **SlimeVR GUI** → Has input field for `arucoMarkerId` and `Optical Reset` button

## 📝 Developer Notes

### Adding a New Marker

```kotlin
// In GUI, set the Marker ID in the input field
tracker.arucoMarkerId = 42

// Everything else happens automatically:
// 1. ArucoProcessor notices new ID
// 2. Creates RingBuffer for history
// 3. On first ArUco packet — calibration happens
// 4. Correction starts applying
```

### Recalibrating a Tracker

```kotlin
// Click "Optical Reset" button in GUI, which calls:
ArucoProcessor.requestOpticReset()

// Then show all markers to camera again for new calibration
```

### Disabling Correction

```kotlin
currentArucoOffsets[markerId] = Quaternion.IDENTITY
```

## 🚀 Performance

- **Memory**: ~100 KB per marker (RingBuffer of 1000 quaternions)
- **CPU**: ~1-2% per marker (PID + history)
- **Latency**: ~20-50 ms (timestamp synchronization)
- **Scalability**: 4-8 markers on single machine without issues
