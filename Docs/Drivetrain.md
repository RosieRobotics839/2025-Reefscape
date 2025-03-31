# DriveTrain Subsystem Documentation

This documentation explains how the DriveTrain subsystem works in our FRC robot, including its helper utilities and SwerveModule components.

## Overview

Our robot uses a swerve drive system, which allows the robot to move in any direction and rotate independently. This provides exceptional maneuverability compared to traditional tank drive or mecanum drive systems. The DriveTrain subsystem coordinates multiple SwerveModules that each control a wheel's speed and direction.

## Core Components

### DriveTrain Class

The main DriveTrain class extends `SubsystemBase` and contains:

- Four SwerveModule instances (frontLeft, frontRight, rearLeft, rearRight)
- Logic to handle field-centric or robot-centric driving
- Position queue for autonomous movement
- PID control for robot heading
- Interfaces to connect joystick inputs to robot movement

```java
public class DriveTrain extends SubsystemBase {
  private static DriveTrain instance = new DriveTrain();
  
  public static DriveTrain getInstance(){
    return instance;
  }

  // Create new swerve modules
  public SwerveModule frontLeft = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_FrontLeft, kDriveTrain.kSwerveModule.kCalibrationFrontLeft, "frontLeft");
  public SwerveModule frontRight = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_FrontRight, kDriveTrain.kSwerveModule.kCalibrationFrontRight, "frontRight");
  public SwerveModule rearLeft = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_RearLeft, kDriveTrain.kSwerveModule.kCalibrationRearLeft, "rearLeft");
  public SwerveModule rearRight = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_RearRight, kDriveTrain.kSwerveModule.kCalibrationRearRight, "rearRight");
  
  // More code...
}
```

### SwerveModule Class

Each SwerveModule controls one wheel's speed and direction, consisting of:

- Drive motor (controls wheel speed)
- Steer motor (controls wheel direction)
- Absolute encoder (for wheel angle feedback)
- Position and velocity control logic

```java
public class SwerveModule extends SubsystemBase {
  public Motor m_motorDrive, m_motorSteer;
  public AnalogInput m_analogEncoder;
  public double angleCalibration;
  public FirstOrderLag m_magLimiter;
  
  // Initialization, state tracking, and control methods
  // ...
}
```

### Motor Class

The Motor class provides a unified interface for different motor controllers (NEO and Kraken):

```java
public class Motor extends SubsystemBase {
  // Motor controller objects (only one will be used)
  public TalonFX motor_talon;
  public MyCANSparkMax motor_neo;
  
  // Control methods and state tracking
  // ...
}
```

## How the Drivetrain Works

### Swerve Drive Kinematics

The drivetrain uses WPILib's `SwerveDriveKinematics` to convert from chassis motion (forward, left, rotate) to individual wheel states (speed and angle).

```java
// Kinematics Definition with Wheel Location Offsets
public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
  new Translation2d(kChassis.kWheelBase / 2, kChassis.kTrackWidth / 2),  // frontLeft
  new Translation2d(kChassis.kWheelBase / 2, -kChassis.kTrackWidth / 2), // frontRight
  new Translation2d(-kChassis.kWheelBase / 2, kChassis.kTrackWidth / 2), // rearLeft
  new Translation2d(-kChassis.kWheelBase / 2, -kChassis.kTrackWidth / 2) // rearRight
);
```

### Control Flow

1. **Input Processing**:
   - The DriveTrain receives inputs either from teleop control or autonomous commands
   - For teleop, inputs come from controllers (FlightStick or XboxController)

2. **Drive Command Processing**:
   - `Drive(double _forward, double _left, double _rotate)` method stores commanded velocities
   - `RunDrive()` method processes commands and handles field-centric transformation 

3. **Movement Calculation**:
   - Forward/left requests are transformed based on robot orientation
   - Heading control is managed by a PID controller for automatic rotation adjustment
   - `kDriveKinematics.toSwerveModuleStates()` converts chassis movement to individual wheel states

4. **Module State Optimization**:
   - Each wheel's target state is optimized with `SwerveModuleState.optimize()` to minimize rotation
   - `SwerveDriveKinematics.desaturateWheelSpeeds()` ensures wheels don't exceed max speed

5. **Command Application**:
   - Each SwerveModule receives its speed and angle targets
   - Modules manage their own PID controllers for precise position and velocity control

### Field-Centric vs. Robot-Centric Driving

The drivetrain supports both control modes:

- **Field-Centric**: The robot's movement is relative to the field, not the robot's heading
  - Forward always moves away from the driver, regardless of robot orientation
  - This is the default and preferred mode for most driving

- **Robot-Centric**: Movement is relative to the robot's current orientation
  - Forward means the direction the robot is facing

```java
// Translation Control Logic
if (OperatorConstants.kFieldCentricDriving){ 
  m_forward = m_maxSpeed * (m_forwardcmd * Math.cos(-m_currentHeading) + m_leftcmd * -Math.sin(-m_currentHeading)); 
  m_left    = m_maxSpeed * (m_forwardcmd * Math.sin(-m_currentHeading) + m_leftcmd *  Math.cos(-m_currentHeading));
} else {
  m_forward = m_maxSpeed * m_forwardcmd;
  m_left    = m_maxSpeed * m_leftcmd;
}
```

### Autonomous Movement

The DriveTrain supports autonomous movement with a queue-based system:

1. Commands add target poses to `m_poseQueue`
2. The periodic loop processes these poses sequentially
3. The robot drives to each pose with speed control and cross-track correction
4. When reaching a pose, it dequeues that pose and moves to the next

```java
// Follow Drive to Pose Queue, unless controller input is active
if (!m_poseQueue.isEmpty()){ 
  Twist2d movement;
  if (VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose, m_poseQueue.peek(), 
                       (m_poseQueue.size() > 1 ? DriveConstants.kMidPointAccuracyFactor : 1) * DriveConstants.kAutoToleranceDistance, 
                       (m_poseQueue.size() > 1 ? DriveConstants.kMidPointAccuracyFactor : 1) * DriveConstants.kAutoToleranceAngle)){
    // Reached Target Pose
    m_poseQueueStart = m_poseQueue.poll();
    movement = new Twist2d(0,0,0);
    Drive(movement);
    nt_distance.set(0);
  } else {
    // Moving to Target Pose
    // ... (calculate movement vector, speed, etc.)
  }
}
```

## Tunable Parameters

### Drive Constants

Located in `kDriveTrain.DriveConstants`:

- **kMaxSpeedMetersPerSecond**: Maximum speed of the drivetrain in m/s
  - Configurable in multiple presets (low, medium, high)
  - Default is typically around 5 feet/second (1.5 m/s)

- **kMaxRotationVelocity**: Maximum rotation speed in rad/s
  - Default around 90 degrees/second

- **kLinearAccelerationTau**: Time constant for acceleration smoothing
  - Lower values = more responsive but potentially jerky
  - Higher values = smoother but less responsive
  - Default is 0.01 seconds

### Heading PID Controller

The heading PID controller maintains the robot's rotation:

- **kPIDHeadingKp**: Proportional gain for heading correction
  - Higher values = stronger correction
  - Default is around 2.0

- **kPIDHeadingKi**: Integral gain to eliminate steady-state error
  - Default is 0 (often not needed)

- **kPIDHeadingKd**: Derivative gain to reduce overshoot
  - Default is 0 (often not needed)

### Swerve Module Constants

Located in `kDriveTrain.kSwerveModule`:

- **kDriveKp, kDriveKi, kDriveKd, kDriveKff**: PID and feedforward gains for drive motor
  - Kraken motors use different values than NEO motors
  - Drive values control velocity accuracy

- **kSteerKp, kSteerKi, kSteerKd, kSteerKff**: PID and feedforward gains for steering motor
  - Steering values control position accuracy
  - Higher Kp = stronger position holding but may oscillate

- **kDrivingMotorCurrentLimit/kSteeringMotorCurrentLimit**: Current limits to protect motors
  - Higher = more torque but potential brownouts
  - Lower = safer operation but less performance

## Tuning Process

### Steer Motor Tuning

1. Start with only Kp (proportional) and set Ki, Kd, and Kff to 0
2. Set Kp to a low value (0.1)
3. Gradually increase Kp until the module turns to position quickly but doesn't overshoot or oscillate
4. If it oscillates, reduce Kp and add a small amount of Kd

### Drive Motor Tuning

1. Start with only Kp and Kff, set Ki and Kd to 0
2. Set Kff to theoretical value (approximately 0.1 for NEO, 0.7 for Kraken)
3. Test driving at various speeds, noting any lag or overshoot
4. Adjust Kp to improve responsiveness 
5. If there's consistent error during acceleration, add small Ki

### Acceleration Limiting

Adjust `kLinearAccelerationTau` for smooth acceleration:
- Too low: Jerky movement and potential wheel slip
- Too high: Sluggish response to driver commands
- Test by quickly changing direction and observing smoothness

## Advanced Features

### Anti-Tipping Protection

The drivetrain has automatic tipping protection:

```java
// If robot is tipping, align the swerve modules in the direction of the tip
if (Gyro.getInstance().isTipping()){
  double speed = VectorUtils.SRSS(FlightStick.forward, FlightStick.left)*m_maxSpeed;
  SwerveModuleState state = new SwerveModuleState(speed,new Rotation2d(Gyro.getInstance().getTippingAngle()));
  forEachSwerveModule((m)->{m.setState(state);});
} else {
  RunDrive();
}
```

### SlowSpeed Control

Motors have a special SlowSpeed mode for precise control at low speeds:

```java
// Check if we should be in SLOWSPEED control mode which uses position control for slow speeds.
if (m_enabSlowSpeed && Math.abs(speed*m_gearReduction) <= MotorDefaults.kSlowThreshold){
  // Compare to last speed target to see if we've reversed direction
  if (Math.signum(m_speedTarget) != Math.signum(speed)){
    m_lowspeedreverse = true;
  }
  m_speedTarget = speed;
  if (m_controlType != ControlType.SLOWSPEED){
    m_controlType = ControlType.SLOWSPEED;
  }
  // Rest of the SlowSpeed logic...
}
```

This helps prevent cogging (uneven movement) at low speeds by switching to position control instead of velocity control.

## Troubleshooting

### Motor Not Responding
- Check if `m_setupDriveDone` and `m_setupSteerDone` are true using NetworkTables
- Verify CAN IDs match in Constants.java
- Check motor controller current limits

### Wheels Misaligned
- Check angle calibration values in Constants.java
- Run calibration procedure again
- Verify encoder connections

### Jerky Movement
- Increase acceleration limiting (kLinearAccelerationTau)
- Lower PID gains, especially Kp and Kd values
- Check for loose mechanical components

### Poor Tracking
- Increase heading PID gains (kPIDHeadingKp)
- Check gyro operation and initialization
- Verify wheel radius and gear ratio constants

## Conclusion

The swerve drive system is complex but powerful, allowing precise control over the robot's movement. Understanding the parameters and tuning them correctly will result in a smooth, responsive driving experience. Experiment with different settings to find what works best for your driving style and competition requirements.
