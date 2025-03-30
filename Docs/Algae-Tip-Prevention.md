# Tip Protection System Documentation

## Overview

The tip protection system is a critical safety feature of our robot that prevents tipping over during operation. When robots with high centers of gravity (like those with arms or elevators) make sudden movements or extend mechanisms too far, they can become unstable. This system detects potential tipping situations and takes automated corrective actions to maintain stability.

## System Architecture

The tip protection system consists of several integrated components:

1. **Tipping Detection** (Gyro Subsystem)
2. **Response Management** (Action Class)
3. **Mechanism Safety Positioning** (Arm and Elevator Subsystems)
4. **Drivetrain Recovery Assistance** (DriveTrain Subsystem)

## 1. Tipping Detection

### Gyro-Based Detection

Tipping is detected using the robot's gyroscope data. Our implementation uses a Pigeon2 IMU to track the robot's orientation and calculate the gravity vector.

```java
// From Gyro.java
public double getTippingAngle(){
    double gvx = -pidgey.getGravityVectorY().getValueAsDouble();
    double gvy = pidgey.getGravityVectorX().getValueAsDouble();
    nt_gvx.set(gvx);
    nt_gvy.set(gvy);
    if (Math.abs(gvx) <= 1e-6 && Math.abs(gvy) <= 1e-6){
      return 0;
    }
    double tipHeading = new Translation2d(gvx,gvy).getAngle().getRadians();
    nt_tipHeading.set(Units.radiansToDegrees(tipHeading));
    return tipHeading;
}
```

**What this does:**
- Reads gravity vector components from the Pigeon IMU
- The gravity vector indicates which way is "down" relative to the robot
- When the robot tilts, the gravity vector's X and Y components change
- The method creates a Translation2d from these components and calculates its angle
- This angle represents the direction the robot is tipping

### Hysteresis Implementation

To prevent rapid oscillation between tipping and non-tipping states, we use a hysteresis approach:

```java
// From Gyro.java
Hysteresis m_isTipped = new Hysteresis()
    .withThreshold(GyroConstants.kTippingAngle)
    .withHysteresis(GyroConstants.kTippingHysteresis);

// In periodic() method
m_isTipped.calculate(m_enableTipDetection ? 
    VectorUtils.SRSS(VectorUtils.angleDifference(0,getPitch()),
    VectorUtils.angleDifference(0,getRoll())) : 0);
```

**What this does:**
- `GyroConstants.kTippingAngle` (typically 12.5 degrees) defines when tipping is detected
- `GyroConstants.kTippingHysteresis` (typically 5 degrees) creates a buffer zone
- The robot must tilt beyond 12.5 degrees to trigger tipping mode
- Once in tipping mode, the tilt must fall below 7.5 degrees (12.5 - 5) to exit tipping mode
- `VectorUtils.SRSS` calculates the Square Root of Sum of Squares (magnitude of tilt)
- `angleDifference` handles angle wrapping to get the correct angular distance

```java
// From Gyro.java
public boolean isTipping(){
    return m_isTipped.get();
}
```

This method exposes the tipping state to other subsystems.

## 2. Response Management

The `Action` class manages state-based responses across subsystems:

```java
// From Action.java
public class Action {
    public boolean m_state;
    
    public boolean m_enable = true;
    public Runnable onTrue;
    public Runnable onFalse;
    public Runnable onChange;
    
    // ...
    
    public void calculate(boolean val){
        if (m_state != val){
            if (m_enable && onChange != null){
                onChange.run();
            }
            if (m_enable && val && onTrue != null){
                onTrue.run();
            }
            if (m_enable && !val && onFalse != null){
                onFalse.run();
            }
            m_state = val;
        }
    }
}
```

**What this does:**
- Allows defining callbacks for state transitions
- Only executes callbacks when the state actually changes
- Provides enable/disable functionality for the entire response

## 3. Mechanism Safety Positioning

### Arm Response

```java
// From Arm.java
Action m_tipProtect = new Action(false).onTrue(()->{
    Controller.getAccessoryInstance().m_directArm=false; 
    moveToLevel(ScoreLevel.FUNNEL);
});

// In periodic()
m_tipProtect.calculate(Gyro.getInstance().isTipping());
```

**What this does:**
- Creates an Action that responds to tipping
- When tipping begins, disables direct driver control of the arm
- Moves the arm to the FUNNEL position (lowest/safest position)
- Calls `calculate()` every cycle with the current tipping state

### Elevator Response

```java
// From Elevator.java
Action m_tipProtect = new Action(false).onTrue(()->{
    Controller.getAccessoryInstance().m_directElevator=false; 
    moveToLevel(ScoreLevel.FUNNEL);
});

// In periodic()
m_tipProtect.calculate(Gyro.getInstance().isTipping());
```

**What this does:**
- Similar to the arm response, creates an Action for the elevator
- Disables direct elevator control when tipping
- Moves the elevator to the lowest position
- Evaluates every cycle based on tipping state

## 4. Drivetrain Recovery Assistance

```java
// From DriveTrain.java
@Override
public void periodic() {
    // ...
    
    // If robot is tipping, align the swerve modules in the direction of the tip
    if (Gyro.getInstance().isTipping()){
        double speed = VectorUtils.SRSS(FlightStick.forward, FlightStick.left)*m_maxSpeed;
        SwerveModuleState state = new SwerveModuleState(
            speed,
            new Rotation2d(Gyro.getInstance().getTippingAngle())
        );
        forEachSwerveModule((m)->{m.setState(state);});
    } else {
        RunDrive();
    }
}
```

**What this does:**
- Checks if the robot is currently tipping
- If tipping, calculates a speed based on driver input
- Gets the tipping angle from the gyro
- Creates a SwerveModuleState with this angle
- Sets all swerve modules to this state
- This allows the robot to drive in the direction of the tip, which helps recovery

## Configuration Parameters

The system's behavior can be tuned through several constants:

```java
// From Constants.java (GyroConstants class)
public static boolean kEnabled = true;
public static double kTippingAngle = Units.degreesToRadians(
    NTDouble.create(12.5,"Gyro/kTippingAngle",
    val->kTippingAngle=Units.degreesToRadians(val))
);
public static double kTippingHysteresis = Units.degreesToRadians(
    NTDouble.create(5,"Gyro/kTippingHysteresis",
    val->kTippingHysteresis=Units.degreesToRadians(val))
);
```

**What this does:**
- `kEnabled` - Master switch for gyro functionality
- `kTippingAngle` - The threshold angle (in degrees) where tipping is detected
- `kTippingHysteresis` - Buffer angle to prevent oscillation
- Both values can be tuned via NetworkTables during operation

## Manual Override

For testing or specific operational needs, tip detection can be manually disabled:

```java
// From Gyro.java
public boolean m_enableTipDetection = true;

// From FlightStick.java
Btm12Btn.onTrue(new InstantCommand(()->Gyro.getInstance().m_enableTipDetection = false));
Btm12Btn.onFalse(new InstantCommand(()->Gyro.getInstance().m_enableTipDetection = true));
```

**What this does:**
- Allows temporarily disabling tip detection via a button (Btm12Btn)
- Useful for situations where tipping is expected and controlled
- Detection automatically re-enables when the button is released

## Integration Points

### Handling Tipping While Driving to Reef

```java
// From Vision.java
var cam2result = processCamera(cam2, photonPoseEstimatorRear, 
    Autonomous.getInstance().m_drivingToReef);
```

**What this does:**
- When driving to the reef during autonomous, ignores certain vision updates
- This prevents conflicting inputs during recovery actions
- Demonstrates how subsystems are aware of each other's states

## Debugging and Telemetry

The system provides telemetry through NetworkTables for debugging:

```java
// From Gyro.java
final DoublePublisher nt_gvx = table.getDoubleTopic("gvx").publish();
final DoublePublisher nt_gvy = table.getDoubleTopic("gvy").publish();
final BooleanPublisher nt_isTipped = table.getBooleanTopic("isTipped").publish();
final DoublePublisher nt_tipHeading = table.getDoubleTopic("tipHeading").publish();

// In periodic()
nt_isTipped.set(m_isTipped.get());
```

**What this does:**
- Publishes gravity vector components
- Shows current tipping state
- Provides the calculated tipping heading
- Allows monitoring system behavior remotely

## Custom Calibration

The system includes provisions for detecting tipping differently based on robot configuration:

```java
// From Gyro.java
m_isTipped.calculate(m_enableTipDetection ? 
    VectorUtils.SRSS(VectorUtils.angleDifference(0,getPitch()),
    VectorUtils.angleDifference(0,getRoll())) : 0);
```

**What this does:**
- The SRSS function takes both pitch and roll into account
- This creates a composite tilt measurement
- Could be modified to weight different axes differently based on robot design

## Summary of Complex Logic

The most sophisticated aspects of the tip protection system include:

1. **Gravity Vector Analysis**: Using gravity vectors provides more information than simple roll/pitch angles. The direction of the gravity vector indicates not just that the robot is tipping, but in which direction.

2. **Stateful Hysteresis**: The hysteresis implementation creates a state machine that prevents oscillation while maintaining responsiveness.

3. **Coordinated Response Across Subsystems**: The system affects multiple subsystems (arm, elevator, drivetrain) in a coordinated fashion.

4. **Drivetrain Recovery Logic**: Rather than simply stopping motors, the system actively assists recovery by aligning wheels in the direction that will help the robot regain stability.

5. **Safe Motion Profiles**: When mechanisms move to safe positions, they follow constrained motion profiles to avoid making tipping worse during the recovery process.

This comprehensive approach makes our tip protection system highly effective at preventing tipping accidents while maintaining operational capability.
