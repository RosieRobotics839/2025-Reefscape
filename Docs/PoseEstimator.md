# PoseEstimator Documentation

## Introduction

The PoseEstimator subsystem is a critical component that helps the robot determine its position and orientation (pose) on the field. It uses multiple sensors—including wheel odometry, gyroscope, and vision data—to create a fused estimate that is more accurate than any single sensor could provide.

This document explains how our PoseEstimator works, its key utilities, tuning parameters, and the mathematics behind the fusion algorithm.

## Core Concepts

### Robot Pose

A robot's pose is described using a `Pose2d` object, which includes:
- X position (meters)
- Y position (meters)
- Rotation (radians)

```java
// Example of creating a pose at the origin facing forward
Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(0));
```

### Pose Estimation Pipeline

The PoseEstimator uses these key inputs:
1. **Wheel Odometry** - Measurement of wheel rotations to estimate movement
2. **Gyroscope** - Measures rotation more accurately than odometry alone
3. **Vision** - Detects AprilTags to provide absolute position measurements

The estimation happens in these steps:
1. Predict pose using wheel odometry
2. Correct prediction using gyroscope data
3. Further correct using vision data (when available)

## Key Components

### Pose Prediction (Odometry)

Odometry uses the SwerveModulePositions to calculate how far the robot has moved:

```java
// Calculate the change in module positions
var moduleDeltas = new SwerveModulePosition[modulePositions.length];
for (int index = 0; index < modulePositions.length; index++) {
    var current = modulePositions[index];
    var previous = m_previousModulePositions[index];

    moduleDeltas[index] = new SwerveModulePosition(
        current.distanceMeters - previous.distanceMeters, 
        current.angle);
    previous.distanceMeters = current.distanceMeters;
}

// Convert to a twist (velocities in x, y, and rotation)
m_predictedTwist = kDriveTrain.kDriveKinematics.toTwist2d(moduleDeltas);
m_predictedTwist = new Twist2d(
    m_predictedTwist.dx * PoseConstants.kDriveSlip, 
    m_predictedTwist.dy * PoseConstants.kDriveSlip, 
    m_predictedTwist.dtheta * PoseConstants.kDriveSlip);

// Use the twist to "move" the pose forward
m_predictedPose = m_finalPose.exp(m_predictedTwist);
```

### Gyroscope Correction

The gyroscope provides more accurate rotation data than wheel odometry:

```java
// Calculate the difference between gyro and estimated angle
double gyroAngle = m_gyro.getYaw();
m_gyroResidual = VectorUtils.angleDifference(gyroAngle, m_tempPose.getRotation().getRadians());

// Apply correction with configured weight
double gyroCorrection = 0;
if (m_gyro.getStatus()) {
    gyroCorrection = m_gyroResidual * (PoseConstants.kGyroWeight);
}

// Update temporary pose with corrected rotation
m_tempPose = new Pose2d(
    m_tempPose.getX(),
    m_tempPose.getY(), 
    new Rotation2d(m_tempPose.getRotation().getRadians() + gyroCorrection));
```

### Vision Correction

Vision uses AprilTags to provide absolute position data:

```java
// Only process new vision data
if (m_visionTimestamp != m_visionLastTimestamp) {
    m_visionIsValid = true;
    m_visionTheta = m_visionPose2d.getRotation();
    
    // Calculate difference between vision and estimated pose
    m_visionPoseResidual = VectorUtils.poseDiff(m_visionPose2d, m_tempPose);
    
    // Apply corrections with configured weights
    m_visionPoseCorrection_pos = m_visionPoseResidual.times(PoseConstants.kVisionWeightPos).getTranslation();
    m_visionPoseCorrection_rot = (m_visionPoseResidual.getRotation().getRadians()) * PoseConstants.kVisionWeightRot;
} else {
    m_visionIsValid = false;
    m_visionTheta = null;
    m_visionPoseCorrection_pos = new Translation2d(0, 0);
    m_visionPoseCorrection_rot = m_visionPoseCorrection_rot * PoseConstants.kVisionWeightRotDecay;
}

// Apply vision corrections to the pose
m_tempPose = new Pose2d(
    m_tempPose.getX() + m_visionPoseCorrection_pos.getX(),
    m_tempPose.getY() + m_visionPoseCorrection_pos.getY(),
    new Rotation2d(m_tempPose.getRotation().getRadians() + m_visionPoseCorrection_rot));
```

## Helper Utilities

### VectorUtils

The `VectorUtils` class provides numerous mathematical functions for working with poses and angles:

```java
// Calculate the minimum difference between two angles
public static double angleDifference(double a1, double a2) {
    return (((a1 % (2*Math.PI)) - (a2 % (2*Math.PI)) + 3*Math.PI) % (2*Math.PI) - Math.PI);
}

// Check if two poses are near each other
public static boolean isNear(Pose2d pose1, Pose2d pose2, double toleranceMeters) {
    if (pose1 == null || pose2 == null) {
        return false;
    }
    Pose2d diff = poseDiff(pose1, pose2);
    return diff.getTranslation().getNorm() < toleranceMeters; 
}

// Find the vector between two poses
public static Pose2d poseDiff(Pose2d pose1, Pose2d pose2) {
    Rotation2d rotation;
    if (pose1.getRotation() != null && pose2.getRotation() != null) {
        rotation = new Rotation2d(angleDifference(
            pose1.getRotation().getRadians(),
            pose2.getRotation().getRadians()
        ));
    } else {
        rotation = new Rotation2d(0);
    }

    return new Pose2d(
        pose1.getX() - pose2.getX(), 
        pose1.getY() - pose2.getY(), 
        rotation);
}
```

### Vision Data Processing

The Vision subsystem processes data from cameras and feeds it to the PoseEstimator:

```java
public void addVisionMeasurement(Optional<EstimatedRobotPose> observedPose, double ts_micros) {
    if (observedPose.isPresent()) {
        m_visionPose3d = observedPose.get().estimatedPose;
        m_visionTimestamp = observedPose.get().timestampSeconds;

        m_visionPose2d = new Pose2d(
            m_visionPose3d.getX(), 
            m_visionPose3d.getY(), 
            m_visionPose3d.getRotation().toRotation2d());
        
        // Calculate latency compensation
        double m_latency = Math.max(0, Math.min(
            VisionConstants.kMaxLatencyCompensationMillis,
            (RobotController.getFPGATime() - ts_micros)/1000.0))/1000.0;
        
        double timeStep = 0.020; 
        
        // Compensate for latency in the vision system
        Twist2d m_latencyCompensation = new Twist2d(
            m_predictedTwist.dx/timeStep*m_latency, 
            m_predictedTwist.dy/timeStep*m_latency, 
            m_predictedTwist.dtheta/timeStep*m_latency);

        // Apply latency compensation
        m_visionPose2d = m_visionPose2d.exp(m_latencyCompensation);
    }
}
```

## Tuning Parameters

The PoseEstimator has several parameters that control the fusion of different data sources:

### `PoseConstants.kDriveSlip` (Default: 1.0)
Adjusts how much odometry measurements are trusted. If wheels slip on the field, decrease this value.
- If robot moves less than expected: Increase above 1.0
- If robot moves more than expected: Decrease below 1.0

### `PoseConstants.kGyroWeight` (Default: 0.10)
Controls how strongly the gyroscope corrects the robot's heading.
- Higher values: Robot heading will snap to gyro reading quickly
- Lower values: Robot heading will change more gradually

### `PoseConstants.kVisionWeightPos` (Default: 0.2)
Controls how strongly vision measurements affect position (X/Y).
- Higher values: Robot position will snap to vision reading
- Lower values: Robot position will change more gradually

### `PoseConstants.kVisionWeightRot` (Default: 0.05)
Controls how strongly vision measurements affect rotation.
- Higher values: Robot heading will snap to vision reading
- Lower values: Robot heading will change more gradually

### `PoseConstants.kVisionWeightRotDecay` (Default: 0.99)
Controls how quickly vision rotation correction decays when no vision is available.
- Values closer to 1.0: Correction persists longer
- Values closer to 0.0: Correction fades quickly

## How to Tune the PoseEstimator

1. **Start with Default Values**: Begin with the default parameters.

2. **Drive Performance**: Drive the robot in teleop and observe its behavior:
   - If the robot drifts while driving straight, adjust `kDriveSlip`
   - If the robot turns more/less than expected, adjust `kGyroWeight`

3. **Vision Performance**: Drive near AprilTags and observe:
   - If position snaps too harshly, reduce `kVisionWeightPos`
   - If heading snaps too harshly, reduce `kVisionWeightRot`
   - If pose doesn't update quickly enough, increase weights

4. **Autonomous Testing**: Run autonomous routines:
   - If robot consistently undershoots targets, increase `kDriveSlip`
   - If robot overshoots, decrease `kDriveSlip`

5. **Consistency Check**: Drive the robot in a known pattern (e.g., square) and check if position estimation matches expected path.

## Advanced Concepts

### Pose Fusion Algorithm

Our PoseEstimator uses a simplified fusion algorithm rather than a full Kalman filter:

1. We start with odometry prediction (`m_predictedPose`)
2. We apply gyro correction with a configurable weight
3. We apply vision correction with configurable weights
4. We decay vision rotation correction over time when no vision updates occur

This provides a good balance of accuracy and computational simplicity.

### Latency Compensation

Vision systems have latency (delay) from when an image is captured to when the pose is calculated. We compensate for this by:

1. Determining the latency (time difference)
2. Calculating how far the robot would have moved during that time
3. Adding that movement to the vision measurement

This makes vision measurements more accurate by accounting for robot movement that occurred during processing.

### Vision Validation

We don't accept all vision measurements, as camera data can sometimes be inaccurate. The `checkVisionPose` method in the Vision class validates measurements by:

1. Checking if the pose is within the field boundary
2. Comparing with recent valid measurements
3. Comparing with odometry-based pose estimate
4. Checking for inconsistent jumps in position

This prevents bad vision data from corrupting our pose estimate.

## Visualization

The PoseEstimator publishes data to both NetworkTables and a Field2d widget:

```java
// Update Field2d visualization
m_field.setRobotPose(m_finalPose);

// Publish to Network Tables
nt_posepublisher.set(m_finalPose);
nt_final_x.set(m_finalPose.getX());
nt_final_y.set(m_finalPose.getY());
nt_final_t.set(m_finalPose.getRotation().getRadians());
```

You can view this in real-time using tools like Shuffleboard or Glass.

## Conclusion

The PoseEstimator combines odometry, gyroscope, and vision data to create a robust position estimate. By understanding how to tune its parameters, you can optimize it for your specific robot and field conditions.

Remember that pose estimation is never perfect—it's always an approximation. But with proper tuning and multiple sensor inputs, it can be accurate enough for autonomous navigation and scoring.
