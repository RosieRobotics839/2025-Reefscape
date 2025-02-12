# Drivetrain System Documentation

## Table of Contents

1. [Overview](#overview)
2. [Components](#components)
    - [Swerve Modules](#swerve-modules)
    - [Motors](#motors)
    - [Encoders](#encoders)
    - [Gyroscope](#gyroscope)
    - [Network Tables](#network-tables)
3. [Kinematics](#kinematics)
    - [Swerve Drive Kinematics](#swerve-drive-kinematics)
    - [Swerve Module State](#swerve-module-state)
4. [Control Loop](#control-loop)
    - [PID Controllers](#pid-controllers)
    - [First Order Lag Filter](#first-order-lag-filter)
5. [Initialization](#initialization)
    - [Drivetrain Initialization](#drivetrain-initialization)
    - [Swerve Module Initialization](#swerve-module-initialization)
6. [Periodic Updates](#periodic-updates)
    - [Driver Inputs](#driver-inputs)
    - [Odometry and Pose Estimation](#odometry-and-pose-estimation)
    - [Motor Commands](#motor-commands)
7. [Autonomous Integration](#autonomous-integration)
    - [Path Planning](#path-planning)
    - [Pose Estimation](#pose-estimation)
8. [Simulation](#simulation)
    - [Swerve Module Simulation](#swerve-module-simulation)
    - [Vision System Simulation](#vision-system-simulation)
9. [Error Handling and Safety](#error-handling-and-safety)
    - [Motor Faults](#motor-faults)
    - [System Monitoring](#system-monitoring)
10. [Constants and Configuration](#constants-and-configuration)
    - [CAN IDs](#can-ids)
    - [Drive Parameters](#drive-parameters)
    - [Gyro Parameters](#gyro-parameters)
    - [Autonomous Parameters](#autonomous-parameters)
11. [Integration with Other Subsystems](#integration-with-other-subsystems)
    - [Vision](#vision)
    - [Autonomous](#autonomous)
12. [Telemetry and Network Tables](#telemetry-and-network-tables)
13. [Conclusion](#conclusion)

---

## Overview

The **Drivetrain System** is responsible for controlling the robot's movement on the field using a swerve drive mechanism. This 
system integrates multiple subsystems, sensors, and control algorithms to achieve precise navigation and maneuverability. Key 
features include:

- **Swerve Drive Modules**: Allow independent rotation and translation of each wheel.
- **PID Controllers**: Ensure accurate motor control for driving and steering.
- **Odometry and Pose Estimation**: Track the robot's position and orientation on the field.
- **Autonomous Integration**: Plan and execute complex paths during the autonomous period.
- **Simulation Support**: Enable drivetrain behavior testing in a simulated environment.
- **Network Tables**: Facilitate real-time telemetry and configuration adjustments.

## Components

### Swerve Modules

Each swerve module comprises a driving motor and a steering motor, each equipped with encoders for precise control. The modules 
are responsible for:

- **Driving**: Translating perception commands into wheel rotations.
- **Steering**: Adjusting wheel angles to achieve desired movement vectors.

### Motors

The drivetrain utilizes two types of motors:

- **TalonFX (KRAKEN)**: Employed for driving the robot, known for high torque and integrated encoders.
- **CANSparkMax (NEO)**: Used for steering the wheels, featuring brushless motors and configurable parameters.

### Encoders

Encoders provide feedback on motor positions and velocities:

- **Driving Encoders**: Track the rotation of driving motors to determine wheel speeds.
- **Steering Encoders**: Monitor wheel angles for accurate steering.

### Gyroscope

A **Pigeon2** gyroscope provides orientation data, essential for:

- **Field-Centric Driving**: Aligning robot movement with the field coordinates.
- **Pose Estimation**: Enhancing odometry accuracy using rotational data.

### Network Tables

Network Tables facilitate communication and telemetry between the robot and external interfaces, enabling:

- **Real-Time Monitoring**: Display drivetrain status and sensor readings.
- **Configuration**: Adjust drivetrain parameters on-the-fly.

## Kinematics

### Swerve Drive Kinematics

The drivetrain employs **Swerve Drive Kinematics**, allowing each wheel to independently steer and drive. This configuration 
offers unparalleled agility, enabling the robot to:

- **Translate**: Move in any direction without rotating.
- **Rotate**: Spin around its center while maintaining position.
- **Maneuver**: Perform complex movements by combining translation and rotation.

**Implementation Details:**

- **Wheel Base and Track Width**: Define the geometric layout of the swerve modules.
- **Translation Vectors**: Calculate desired movement vectors based on driver inputs and autonomous commands.
- **Desaturation**: Normalize wheel speeds to prevent exceeding maximum achievable speeds.

### Swerve Module State

Each swerve module maintains a **SwerveModuleState**, encapsulating:

- **Speed (meters per second)**: Specifies the target speed of the driving motor.
- **Angle (Rotation2d)**: Defines the target steering angle of the wheel.

**Optimization:**

- **State Optimization**: Adjust wheel angles to minimize rotation from the current position.
  
```java
public void setState(SwerveModuleState targetState) {
    if (!m_setupDriveDone || !m_setupSteerDone) return;
    // Sets the target state of the swerve drive equal to the input state
    targetState.optimize(new Rotation2d(m_motorSteer.getPosition()));
    optimizedState = targetState;
}
```

## Control Loop

### PID Controllers

The drivetrain utilizes **PID Controllers** to manage both driving and steering motors. These controllers ensure that the 
motors achieve and maintain their target states with high precision.

- **Proportional (P)**: Reacts to the current error.
- **Integral (I)**: Addresses accumulated past errors.
- **Derivative (D)**: Predicts future errors based on rate of change.

**Drive Motor PID Configuration:**

```java
m_motorDrive
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit((int)kSwerveModule.kDrivingMotorCurrentLimit)
    .positionConversionFactor((Robot.isSimulation() ? 60: kSwerveModule.kDriveEncoderPositionFactor))
    .velocityConversionFactor(kSwerveModule.kDriveEncoderVelocityFactor)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(kSwerveModule.kDriveKp, kSwerveModule.kDriveKi, kSwerveModule.kDriveKd, kSwerveModule.kDriveKff)
    .outputRange(-1,1)
    .iZone(0.15);
```

**Steering Motor PID Configuration:**

```java
m_motorSteer
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit((int)kSwerveModule.kSteeringMotorCurrentLimit)
    .positionConversionFactor(kSwerveModule.kSteerEncoderPositionFactor)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(kSwerveModule.kSteerKp, kSwerveModule.kSteerKi, kSwerveModule.kSteerKd, kSwerveModule.kSteerKff)
    .outputRange(-1,1)
    .iZone(0.05)
    .positionWrappingEnabled(true)
    .positionWrappingConfig(-Math.PI, Math.PI)
    .setCalibration(m_steeringOffset - angleCalibration);
```

### First Order Lag Filter

A **First Order Lag Filter** is applied to the driving speed commands to smooth accelerations, preventing abrupt changes that 
could destabilize the robot.

```java
public FirstOrderLag m_magLimiter = new FirstOrderLag(DriveConstants.kLinearAccelerationTau, 0, 0.020);
```

## Initialization

### Drivetrain Initialization

Upon robot startup, each swerve module is initialized with specific configurations, including motor inversion, idle modes, 
current limits, and PID constants.

```java
public DriveTrain() {
    m_headingPID = new PIDController(DriveConstants.kPIDHeadingKp, DriveConstants.kPIDHeadingKi, DriveConstants.kPIDHeadingKd);
    // Additional initialization code...
}
```

### Swerve Module Initialization

Each swerve module (e.g., `frontLeft`, `frontRight`, `rearLeft`, `rearRight`) is instantiated with unique CAN IDs and angle 
calibration values. During initialization:

1. **Motor Configuration**: Set motor inversion, idle modes, current limits, and PID constants.
2. **Encoder Calibration**: Adjust steering offsets based on encoder readings and calibration data.
3. **State Setting**: Initialize modules to a default state to ensure synchronized movement.

```java
public SwerveModule(CANID_t CANID, double angleCalibration, String name) {
    // Motor Drive Initialization
    m_motorDrive = new Motor(CANID.driving, Motor.MyMotorType.KRAKEN, name+"_driving");
    m_motorDrive
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int)kSwerveModule.kDrivingMotorCurrentLimit)
        .positionConversionFactor((Robot.isSimulation() ? 60: kSwerveModule.kDriveEncoderPositionFactor))
        .velocityConversionFactor(kSwerveModule.kDriveEncoderVelocityFactor)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kSwerveModule.kDriveKp, kSwerveModule.kDriveKi, kSwerveModule.kDriveKd, kSwerveModule.kDriveKff)
        .outputRange(-1,1)
        .iZone(0.15);
    
    // Steering Motor Initialization
    m_motorSteer = new Motor(CANID.steering, Motor.MyMotorType.NEO, name+"_steering");
    m_motorSteer
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int)kSwerveModule.kSteeringMotorCurrentLimit)
        .positionConversionFactor(kSwerveModule.kSteerEncoderPositionFactor)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kSwerveModule.kSteerKp, kSwerveModule.kSteerKi, kSwerveModule.kSteerKd, kSwerveModule.kSteerKff)
        .outputRange(-1,1)
        .iZone(0.05)
        .positionWrappingEnabled(true)
        .positionWrappingConfig(-Math.PI, Math.PI)
        .setCalibration(m_steeringOffset - angleCalibration);
    
    setState(optimizedState);
}
```

## Periodic Updates

### Driver Inputs

During teleoperated mode, the drivetrain processes inputs from the driver’s controller to determine movement commands.

- **Translation**: Forward and left inputs translate to movement vectors.
- **Rotation**: Right joystick X-axis controls rotation.
- **Speed Limiting**: Driver inputs are processed through a First Order Lag filter to smooth acceleration.

```java
public void Translate() {
    double flipField = 1.0;
    if (!m_blueAlly){
        flipField = -1.0;
    }
    
    slider = this.getThrottle();

    Translation2d Lstick = new Translation2d(-this.getX(),-this.getY());
    Lstick = VectorUtils.deadband(Lstick,0.1,1);
    forward = flipField * turboscale * Lstick.getY();
    left = flipField * Lstick.getX();
    
    Translation2d Rstick = new Translation2d(this.getZ(),0);
    Rstick = VectorUtils.deadband(Rstick,0.35,1);
    rotate = -Rstick.getX();
}
```

### Odometry and Pose Estimation

The drivetrain integrates with the **PoseEstimator** subsystem to track the robot’s position on the field using:

- **Swerve Module Positions**: Forward kinematics calculates the robot's movement based on wheel movements.
- **Gyroscope Data**: Enhances rotational accuracy for better pose estimation.
- **Vision System (Optional)**: Adjusts pose based on visual targets for improved accuracy.

```java
Pose2d diff = VectorUtils.poseDiff(m_poseQueue.peek(),PoseEstimator.getInstance().m_finalPose);
double distance = 
Math.max(0,VectorUtils.poseDiff(m_poseQueue.peekLast(),PoseEstimator.getInstance().m_finalPose).getTranslation().getNorm() - 
kChassis.kWheelBase/2.0);
m_autoSpeed = 
m_autoAccelLimiter.calculate(Math.max(Math.min(1,distance/(DriveConstants.kAutoSlowDist))*DriveConstants.kAutoMaxSpeed, 
DriveConstants.kAutoMinSpeed));
Translation2d vector = VectorUtils.vectorInDirectionOf(diff, m_autoSpeed);
movement = new Twist2d(vector.getX(), vector.getY(), 0);
```

### Motor Commands

Based on computed states, the drivetrain sends commands to each swerve module to achieve desired movement.

1. **State Calculation**: Convert desired chassis speeds to individual module states.
2. **Desaturation**: Normalize wheel speeds to prevent exceeding maximum speeds.
3. **Set Module States**: Apply calculated states to each swerve module.

```java
// Calculate new Swerve Module states using Reverse Kinematics
swerveModuleStates = kDriveTrain.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(translationReq.getX(), 
translationReq.getY(), m_rotate));

// Keep direction control and slow down if exceeding maximum wheel speed.
SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kAttainableMaxSpeed);

frontLeft.setState(swerveModuleStates[0]);
frontRight.setState(swerveModuleStates[1]);
rearLeft.setState(swerveModuleStates[2]);
rearRight.setState(swerveModuleStates[3]);
```

## Autonomous Integration

### Path Planning

The drivetrain integrates with the **PathPlanning** subsystem to execute autonomous paths. Using A* pathfinding on a predefined 
graph of field poses, the drivetrain can navigate to specified waypoints.

```java
public void navigateTo(Pose2d to){
    Pose2d from;
    if (DriveTrain.getInstance().m_poseQueue.size() == 0){
        from = PoseEstimator.getInstance().m_finalPose;
    } else {
        from = DriveTrain.getInstance().m_poseQueue.getLast();
    }
    navigateTo(to, from);
}
```

### Pose Estimation

**PoseEstimator.java** works in tandem with the drivetrain to maintain an accurate representation of the robot's position.

- **Swerve Module Data**: Provides positional data for odometry.
- **Gyroscope Data**: Enhances rotational tracking.
- **Vision Data**: Optionally corrects pose based on visual inputs.

```java
public class PoseEstimator extends SubsystemBase {
    // Pose Estimation Logic...
    public void addVisionMeasurement(Optional<EstimatedRobotPose> observedPose, double latency_ms){
        if (observedPose.isPresent()){
            // Process and apply vision correction...
        }
    }
}
```

## Simulation

### Swerve Module Simulation

In a simulated environment, the drivetrain's behavior is mirrored using simulated motor models and sensor feedback. This allows 
for:

- **Testing**: Validate drivetrain responses without physical hardware.
- **Tuning**: Adjust PID parameters and controls in a controlled setting.

```java
public void simulationInit(){
    m_simDrive = new SparkMaxSim(m_motorDrive.motor_neo, m_simMotorDrive);
    m_simSteer = new SparkMaxSim(m_motorSteer.motor_neo, m_simMotorSteer);
}

public void simulationPeriodic(){
    if (!m_setupDriveDone || !m_setupSteerDone) return;
  
    if (!RobotController.isSysActive()){
        m_motorDrive.setSpeed(0);
    }
}
```

### Vision System Simulation

**Vision.java** includes simulation support for the vision system, enabling testing of pose estimation and target acquisition 
algorithms.

```java
@Override
public void simulationPeriodic() {
    simVision.update(PoseEstimator.getInstance().m_sim_actualPose);
}
```

## Error Handling and Safety

### Motor Faults

The drivetrain monitors motor statuses to detect and respond to faults. In case of a hardware fault:

1. **Detection**: Monitor fault flags from motor controllers.
2. **Recovery**: Attempt to reset motors after specific intervals.
3. **Safety**: Prevent drivetrain operation until faults are resolved.

```java
if (pidgey.getFault_Hardware().getValue() && m_lastReset == 0){
    pidgey.reset();
    m_lastReset = 5;
    initypr = new double []{0,0,0};
}
```

### System Monitoring

Utilizing **Network Tables**, drivetrain status and sensor data are continuously published for real-time monitoring:

- **Gyro Status**: Indicates if the gyroscope is functioning correctly.
- **Motor Currents**: Tracks driving and steering motor currents.
- **Pose Data**: Displays current and predicted poses.

```java
public void periodic() {
    nt_status.set(getStatus());
    nt_yaw.set(Units.radiansToDegrees(ypr[0]));
    nt_pitch.set(Units.radiansToDegrees(ypr[1]));
    nt_roll.set(Units.radiansToDegrees(ypr[2]));
    // Additional telemetry...
}
```

## Constants and Configuration

### CAN IDs

Defines unique identifiers for each motor and sensor on the CAN bus, ensuring proper communication and control.

```java
public static class CANID_t {
    public int encoder, driving, steering;
    public CANID_t(int abs_encoder, int drive, int steer){
        this.encoder = abs_encoder;
        this.driving = drive;
        this.steering = steer;
    }
}
```

### Drive Parameters

Includes parameters such as maximum speeds, acceleration limits, and drive geometries.

```java
public static class DriveConstants {
    public static double kLinearAccelerationTau = 0.01;
    public static double kMaxSpeed = Units.feetToMeters(4);
    public static double kMaxRotationVelocity = Units.degreesToRadians(75);
    // Additional constants...
}
```

### Gyro Parameters

Configuration parameters for the gyroscope, including calibration and maximum correction rates.

```java
public static class GyroConstants{
    public static double kVisionCorrectionMaxRate = Units.degreesToRadians(40);
    public static int kCANID_Pigeon = 50;
}
```

### Autonomous Parameters

Defines parameters specific to autonomous operations, such as target distances, angle tolerances, and path planning 
configurations.

```java
public static class AutoConstants {
    public static double kSpeakerNearDistance = Units.feetToMeters(6.5);
    public static double kMaxLatencyCompensationMillis = 200;
    // Additional constants...
}
```

## Integration with Other Subsystems

### Vision

The drivetrain leverages the **Vision** subsystem to enhance pose estimation and enable targeting based on visual inputs.

- **Pose Correction**: Adjusts odometry based on visual target detections.
- **Target Locking**: Maintains focus on specific vision targets during operation.

```java
if (res.hasTargets()) {
    Optional<EstimatedRobotPose> robotPose = photonPoseEstimatorFront.update(res, cam1.getCameraMatrix(), 
cam1.getDistCoeffs());
    if (robotPose.isPresent()){
        Pose3d camPose = robotPose.get().estimatedPose;
        PoseEstimator.getInstance().addVisionMeasurement(robotPose, VisionConstants.kExtraLatencyCompensationMillis + 
res.getLatencyMillis());
    }
}
```

### Autonomous

During the autonomous period, the drivetrain collaborates with the **PathPlanning** and **PoseEstimator** subsystems to 
navigate predefined paths and execute autonomous commands.

```java
public void navigateTo(Pose2d to){
    Pose2d from = DriveTrain.getInstance().m_poseQueue.isEmpty() ? PoseEstimator.getInstance().m_finalPose : 
DriveTrain.getInstance().m_poseQueue.getLast();
    // Path finding and navigation logic...
}
```

## Telemetry and Network Tables

The drivetrain extensively utilizes **Network Tables** to publish telemetry data and receive configuration inputs. This 
integration facilitates:

- **Real-Time Monitoring**: Displays drivetrain states on dashboards like SmartDashboard or Shuffleboard.
- **Dynamic Configuration**: Adjust PID constants and other parameters without redeploying code.
- **Diagnostic Information**: Provides insights into drivetrain performance and potential issues.

```java
nt_final_x.set(m_finalPose.getX());
nt_final_y.set(m_finalPose.getY());
nt_final_t.set(m_finalPose.getRotation().getRadians());
nt_speed.set(translationReq.getNorm());
nt_rotate.set(m_rotate);
```

## Conclusion

The drivetrain system is a sophisticated integration of hardware components, control algorithms, and software subsystems 
designed to provide precise and agile movement capabilities. By leveraging swerve drive kinematics, advanced PID control loops, 
and real-time telemetry, this system enables the robot to navigate the field effectively both in teleoperated and autonomous 
modes. Continuous monitoring and simulation support further enhance reliability and performance, ensuring competitive edge 
during matches.

---

# Appendix

## Code Snippets

### DriveTrain.java

```java
public class DriveTrain extends SubsystemBase {
    // Singleton Instance
    private static DriveTrain instance = new DriveTrain();
    public static DriveTrain getInstance(){
        return instance;
    }

    // Swerve Modules
    public SwerveModule frontLeft = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_FrontLeft, 
kDriveTrain.kSwerveModule.kCalibrationFrontLeft, "frontLeft");
    public SwerveModule frontRight = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_FrontRight, 
kDriveTrain.kSwerveModule.kCalibrationFrontRight, "frontRight");
    public SwerveModule rearLeft = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_RearLeft, 
kDriveTrain.kSwerveModule.kCalibrationRearLeft, "rearLeft");
    public SwerveModule rearRight = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_RearRight, 
kDriveTrain.kSwerveModule.kCalibrationRearRight, "rearRight");

    // Other Components
    public PIDController m_headingPID;
    LinkedList<Pose2d> m_poseQueue = new LinkedList<Pose2d>();

    // Initialization
    private DriveTrain() {
        m_headingPID = new PIDController(DriveConstants.kPIDHeadingKp, DriveConstants.kPIDHeadingKi, 
DriveConstants.kPIDHeadingKd);
        m_headingPID.publish(table,"HeadingPID");
        m_headingPID.setNTScale(Units.radiansToDegrees(1));
        m_headingPID.setIntegratorRange(-DriveConstants.kPIDHeadingIntegratorRange, DriveConstants.kPIDHeadingIntegratorRange);
        m_headingPID.enableContinuousInput(0, Units.degreesToRadians(360));
    }

    // Periodic Updates
    @Override
    public void periodic() {
        // Handle motor setup checks, driver inputs, autonomous navigation
        RunDrive();
    }

    // Drive Control
    public void RunDrive() {
        // Autonomous and teleop drive logic
        // Calculate movement vectors, apply desaturation, set swerve module states
    }

    // Helper Methods
    public static void forEachSwerveModule(Consumer<SwerveModule> lambda){
        var dt = DriveTrain.getInstance();
        SwerveModule [] motors = {dt.frontLeft, dt.frontRight, dt.rearLeft, dt.rearRight};
        for (SwerveModule m : motors){
            lambda.accept(m);
        }
    }
}
```

### SwerveModule.java

```java
public class SwerveModule extends SubsystemBase {
    public Motor m_motorDrive, m_motorSteer;
    SwerveModuleState optimizedState = new SwerveModuleState(0, new Rotation2d(0));
    FirstOrderLag m_magLimiter = new FirstOrderLag(DriveConstants.kLinearAccelerationTau, 0, 0.020);
    // Network Table Publishers
    DoublePublisher nt_anglecmd, nt_speedcmd;

    public SwerveModule(CANID_t CANID, double angleCalibration, String name) {
        // Initialize Drive Motor
        m_motorDrive = new Motor(CANID.driving, Motor.MyMotorType.KRAKEN, name+"_driving");
        m_motorDrive.configure(...); // Apply configurations

        // Initialize Steering Motor
        m_motorSteer = new Motor(CANID.steering, Motor.MyMotorType.NEO, name+"_steering");
        m_motorSteer.configure(...); // Apply configurations

        setState(optimizedState);
    }

    public void setState(SwerveModuleState targetState) {
        if (!m_setupDriveDone || !m_setupSteerDone) return;
        targetState.optimize(new Rotation2d(m_motorSteer.getPosition()));
        optimizedState = targetState;
    }

    public void setSpeed(double speedMetersPerSecond){
        if (!m_setupDriveDone) return;
        optimizedState.speedMetersPerSecond = speedMetersPerSecond;
        m_motorDrive.setSpeed(speedMetersPerSecond);
    }

    @Override
    public void periodic() {
        // Update motor commands based on optimized state
        if (m_setupDriveDone && m_setupSteerDone){
            m_motorDrive.setSpeed(speedcmd);
            m_motorSteer.setPosition(anglecmd);
            // Publish telemetry
        }
    }
}
```

---

# Glossary

- **PID Controller**: A control loop mechanism employing proportional, integral, and derivative components to maintain desired 
system states.
- **Slew Rate Limiter**: A filter that restricts the rate of change of a signal, helping to smooth transitions.
- **Odometry**: The use of data from motion sensors to estimate changes in position over time.
- **Pose**: Represents the robot's position and orientation on the field.
- **Swerve Drive**: A drivetrain configuration where each wheel can rotate independently, allowing omnidirectional movement.
- **Network Tables**: A network-based key-value store used for communication between robot code and external interfaces.

# References

- [WPILib Documentation](https://docs.wpilib.org/)
- [PhotonVision Documentation](https://docs.photonvision.org/en/latest/)
- [FIRST Robotics Competition Rules](https://www.firstinspires.org/legal/frc-rules)

---

