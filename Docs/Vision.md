# Vision Subsystem Documentation

## Overview

The Vision subsystem enables the robot to see and understand its surroundings using cameras and AprilTags. This documentation explains how the Vision system works, its main components, and how to fine-tune it for optimal performance.

## Main Components

### 1. PhotonVision Cameras

Our robot uses two cameras for vision processing:
- **Front Camera** (`cam1`): Mounted on the front of the robot
- **Rear Camera** (`cam2`): Mounted on the back of the robot

These cameras detect AprilTags on the field, which are special markers placed at known locations.

### 2. AprilTag Field Layout

The field layout contains the positions of all AprilTags on the field:

```java
public AprilTagFieldLayout aprilTagFieldLayout;

// Loaded from a JSON file
try {
  aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.kFieldLayout);
  PublishAprilTags();
} catch (IOException e) {
  e.printStackTrace();
}
```

### 3. Pose Estimators

Each camera has its own pose estimator that calculates the robot's position on the field:

```java
public static PhotonPoseEstimator photonPoseEstimatorFront, photonPoseEstimatorRear;

// Initialization
photonPoseEstimatorFront = new PhotonPoseEstimator(
    aprilTagFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
    VisionConstants.frontCamera.kCameraToRobot().inverse()
);
```

## How Vision Processing Works

### Camera Result Processing

The `processCamera()` method is the heart of vision processing:

```java
private CamResult processCamera(PhotonCamera camera, final PhotonPoseEstimator poseEst, boolean ignore) {
    // Get all unread results from the camera
    var results = camera.getAllUnreadResults();
    
    // Start from the most recent result
    ListIterator<PhotonPipelineResult> res_iter = results.listIterator(results.size());

    while (res_iter.hasPrevious()) {
        PhotonPipelineResult result = res_iter.previous();
        
        if (result.hasTargets()) {
            // Process targets and estimate robot pose
            // ...
        }
    }
}
```

This method:
1. Gets the latest camera results
2. Checks if AprilTags are detected
3. Estimates the robot's position based on the detected tags
4. Validates the pose estimate
5. Updates the robot's position estimate if valid

### Vision Data Flow

1. Cameras capture images of the field
2. PhotonVision processes these images to detect AprilTags
3. Our Vision subsystem estimates the robot's position from the AprilTag data
4. The PoseEstimator subsystem combines this with other sensor data (gyro, odometry)
5. The final pose is used for autonomous navigation and driver assistance

## Pose Validation

Not all vision measurements are accurate. The `checkVisionPose()` method filters out bad pose estimates:

```java
private boolean checkVisionPose(EstimatedRobotPose robotPose) {
    // Check if pose is within field boundaries
    if (!poseIsInField(robotPose.estimatedPose.toPose2d())) {
        return false;
    }
    
    // Check if pose is consistent with recent measurements
    if (m_lastVisionPose != null && 
        robotPose.timestampSeconds - m_lastVisionPose.timestampSeconds < 0.5) {
        if (!VectorUtils.isNear(robotPose.estimatedPose.toPose2d(), 
                               m_lastVisionPose.estimatedPose.toPose2d(), 1)) {
            return false;
        }
    }
    
    // More validation checks...
    
    return true;
}
```

## Tunable Parameters

### 1. Camera Positioning Constants

```java
// In VisionConstants class
public static class frontCamera {
    public static final String kCameraName = "FrontCam";
    public static double kCamYawRight = Math.toRadians(15);
    public static double kCamPitchUp = Math.toRadians(0);
    public static double kCamForwardOffset = Units.inchesToMeters(6.875);
    public static double kCamLeftOffset = Units.inchesToMeters(7.375);
}
```

These parameters define the camera's position and orientation relative to the robot center:
- `kCamYawRight`: Camera's rotation around the vertical axis (in degrees)
- `kCamPitchUp`: Camera's tilt angle
- `kCamForwardOffset`: How far forward the camera is mounted from the robot center
- `kCamLeftOffset`: How far left the camera is mounted from the robot center

**Tuning**: Measure these values carefully on the robot. Incorrect measurements lead to poor pose estimates.

### 2. Vision Filtering Parameters

```java
// Maximum ambiguity allowed for a vision measurement to be accepted
public static double kMaxAmbiguity = 0.2;

// Maximum allowable latency for vision processing (milliseconds)
public static double kMaxLatencyCompensationMillis = 200;
```

- `kMaxAmbiguity`: Lower values (closer to 0) are more strict, requiring more confident AprilTag detections
- `kMaxLatencyCompensationMillis`: Maximum time difference allowed between image capture and processing

**Tuning**:
- If vision is unreliable but detects tags, decrease `kMaxAmbiguity`
- If vision is too strict and ignores valid tags, increase `kMaxAmbiguity` slightly

### 3. Pose Residual Thresholds

In the LED class, these parameters control when to indicate vision issues:

```java
public static double kPoseResidualDist = Units.feetToMeters(2.0);
public static double kPoseResidualRot = Units.degreesToRadians(30);
```

These thresholds determine how much disagreement between vision and odometry is acceptable.

**Tuning**: If the LEDs frequently flash indicating vision issues, these values may need adjustment.

## Vision Weights in Pose Estimator

The Vision subsystem provides data to the PoseEstimator, which combines it with other sensors:

```java
// In PoseConstants class
public static double kVisionWeightRotDecay = 0.99;
public static double kVisionWeightRot = 0.05;
public static double kVisionWeightPos = 0.2;
```

- `kVisionWeightPos`: How much to trust vision for position (X,Y) updates (0.0-1.0)
- `kVisionWeightRot`: How much to trust vision for rotation updates (0.0-1.0)
- `kVisionWeightRotDecay`: How quickly to reduce rotation corrections when no new vision data is available

**Tuning**:
- Higher values make the robot more responsive to vision updates but might make it jumpy
- Lower values make pose estimation smoother but less responsive to vision corrections
- Start with small values (0.05-0.2) and increase if vision is very reliable

## Simulation Support

The Vision subsystem includes simulation support for testing without physical hardware:

```java
if (Robot.isSimulation()) {
    simVision = new VisionSystemSim(VisionConstants.frontCamera.kCameraName);
    
    simCameraProperties = new SimCameraProperties();
    simCameraProperties.setCalibration(
        VisionConstants.frontCamera.kSimCamResolutionW,
        VisionConstants.frontCamera.kSimCamResolutionH,
        new Rotation2d(VisionConstants.frontCamera.kCamDiagFOV)
    );
    
    // More simulation setup...
}
```

## Debugging Vision

The Vision subsystem publishes data to NetworkTables that can be viewed in Shuffleboard:

```java
nt_posefront.set(cam1result);
nt_cam1IsConnected.set(cam1.isConnected());
nt_cam2IsConnected.set(cam2.isConnected());
```

Key values to monitor:
- Camera connection status
- Number of targets detected
- Pose estimates (X, Y, rotation)
- Ambiguity values of detected tags

## Troubleshooting

1. **No AprilTags detected**:
   - Check camera connections
   - Verify cameras are in the right pipeline mode (use `cam1.setPipelineIndex()`)
   - Make sure cameras have a clear view of the field

2. **Inconsistent pose estimates**:
   - Check camera mounting (are the constants correct?)
   - Verify field layout file is correct
   - Increase filtering strictness with `kMaxAmbiguity`

3. **Robot position jumps suddenly**:
   - Decrease vision weights (`kVisionWeightPos` and `kVisionWeightRot`)
   - Improve pose validation in `checkVisionPose()`

4. **Vision works in some field areas but not others**:
   - Check which AprilTags are being detected in different areas
   - Consider camera orientation and field layout

## Advanced Feature: Field Layout Switching

The system supports switching between different field layouts:

```java
public void reloadFieldLayout() {
    try {
        // Update field layout path
        String fieldLayoutPath = VisionConstants.getFieldLayoutPath();
        
        // Load new field layout
        aprilTagFieldLayout = new AprilTagFieldLayout(fieldLayoutPath);
        
        // Update pose estimators
        photonPoseEstimatorFront.setFieldTags(aprilTagFieldLayout);
        photonPoseEstimatorRear.setFieldTags(aprilTagFieldLayout);
        
        // Update simulation if in sim mode
        if (Robot.isSimulation() && simVision != null) {
            simVision.clearAprilTags();
            simVision.addAprilTags(aprilTagFieldLayout);
        }
    } catch (IOException e) {
        e.printStackTrace();
    }
}
```

This is useful for switching between practice and competition field layouts.

## Summary

The Vision subsystem provides the robot with spatial awareness by detecting AprilTags and estimating the robot's position. By tuning the parameters described above, you can optimize the balance between responsiveness and stability in vision-based positioning.
