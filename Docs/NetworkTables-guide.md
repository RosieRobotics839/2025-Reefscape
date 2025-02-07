# Publishing Telemetry to Network Tables in FRC Robot Code

Network Tables is a key-value storage system that allows for real-time data communication between the robot and driver station (typically through dashboards like SmartDashboard or Shuffleboard). It's extensively used in FRC (FIRST Robotics Competition) to publish various telemetry data, enabling teams to monitor robot performance, debug issues, and make informed decisions during matches.

This guide will walk you through the basics of publishing telemetry data to Network Tables with examples demonstrating different data types.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Understanding Network Tables](#understanding-network-tables)
3. [Setting Up Network Tables](#setting-up-network-tables)
4. [Publishing Different Data Types](#publishing-different-data-types)
   - [Publishing Doubles](#publishing-doubles)
   - [Publishing Booleans](#publishing-booleans)
   - [Publishing Arrays](#publishing-arrays)
5. [Example Implementations](#example-implementations)
6. [Best Practices](#best-practices)
7. [Additional Resources](#additional-resources)

## Prerequisites

- **FRC Robot Project Setup**: Ensure you have an FRC robot project set up with access to WPILib libraries.
- **Familiarity with Java**: Basic understanding of Java programming is assumed.

## Understanding Network Tables

Network Tables operate on a client-server model where the robot acts as a server, and dashboards or driver station applications act as clients. Data published by the robot can be instantly accessed and visualized on client applications.

Key concepts:

- **NetworkTableInstance**: Represents an instance of Network Tables.
- **NetworkTable**: A table within the instance that contains key-value pairs.
- **Publishers and Subscribers**: Publishers send data to Network Tables, while subscribers receive updates.

## Setting Up Network Tables

Before publishing data, initialize and retrieve the appropriate `NetworkTable`.

```java
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Get the default instance of Network Tables
NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

// Retrieve a specific table, e.g., "robot_telemetry"
NetworkTable telemetryTable = ntInstance.getTable("robot_telemetry");
```

## Publishing Different Data Types

### Publishing Doubles

Doubles are commonly used for continuous telemetry like sensor readings or calculated values.

```java
import edu.wpi.first.networktables.DoublePublisher;

// Create a publisher for a double value, e.g., "sensor_distance"
DoublePublisher sensorDistancePublisher = telemetryTable.getDoubleTopic("sensor_distance").publish();

// To publish/update the value:
double distance = 3.5; // Example sensor reading in meters
sensorDistancePublisher.set(distance);
```

### Publishing Booleans

Booleans are useful for binary states such as subsystem statuses or condition flags.

```java
import edu.wpi.first.networktables.BooleanPublisher;

// Create a publisher for a boolean value, e.g., "drivetrain_active"
BooleanPublisher drivetrainActivePublisher = telemetryTable.getBooleanTopic("drivetrain_active").publish();

// To publish/update the value:
boolean isActive = true; // Example status
drivetrainActivePublisher.set(isActive);
```

### Publishing Arrays

Arrays can store multiple related values, such as positional data or a series of sensor readings.

```java
import edu.wpi.first.networktables.DoubleArrayPublisher;

// Create a publisher for a double array, e.g., "wheel_speeds"
DoubleArrayPublisher wheelSpeedsPublisher = telemetryTable.getDoubleArrayTopic("wheel_speeds").publish();

// To publish/update the array:
double[] wheelSpeeds = {1.2, 1.5, 1.3, 1.4}; // Speeds for four wheels in m/s
wheelSpeedsPublisher.set(wheelSpeeds);
```

## Example Implementations

Below are examples extracted and adapted from our robot code, demonstrating how to publish different telemetry data types.

### Example 1: Publishing Double Values

**Class**: `Gyro.java`

The `Gyro` subsystem publishes yaw, pitch, and roll angles to Network Tables.

```java
import edu.wpi.first.networktables.DoublePublisher;

// Within the Gyro class constructor
DoublePublisher nt_yaw = table.getDoubleTopic("yaw").publish();
DoublePublisher nt_pitch = table.getDoubleTopic("pitch").publish();
DoublePublisher nt_roll = table.getDoubleTopic("roll").publish();

// In the periodic method, update the publishers with new gyro data
public void periodic() {
    double[] newypr = getypr(); // Method to retrieve yaw, pitch, roll
    nt_yaw.set(newypr[0]);
    nt_pitch.set(newypr[1]);
    nt_roll.set(newypr[2]);
}
```

### Example 2: Publishing Boolean Values

**Class**: `SystemLog.java`

The `SystemLog` subsystem publishes various status flags to Network Tables.

```java
import edu.wpi.first.networktables.BooleanPublisher;

// Within the SystemLog class constructor
BooleanPublisher nt_isEnabled = table.getBooleanTopic("isEnabled").publish();
BooleanPublisher nt_isEStopped = table.getBooleanTopic("isEStopped").publish();

// In the periodic method, update the publishers with system statuses
public void periodic() {
    nt_isEnabled.set(DriverStation.isEnabled());
    nt_isEStopped.set(DriverStation.isEStopped());
}
```

### Example 3: Publishing Arrays

**Class**: `DriveTrain.java`

The `DriveTrain` subsystem publishes the states of all swerve modules as an array.

```java
import edu.wpi.first.networktables.DoubleArrayPublisher;

// Within the DriveTrain class
DoubleArrayPublisher nt_module_states = table.getDoubleArrayTopic("swerve_module_states").publish();

// In the periodic method, update the array with current module states
public void periodic() {
    double[] moduleStates = {
        frontLeft.getState().speedMetersPerSecond,
        frontRight.getState().speedMetersPerSecond,
        rearLeft.getState().speedMetersPerSecond,
        rearRight.getState().speedMetersPerSecond
    };
    nt_module_states.set(moduleStates);
}
```

## Best Practices

1. **Consistent Naming Conventions**: Use clear and consistent names for your Network Tables to avoid confusion. For example, use `"robot_telemetry/sensor_distance"` instead of `"distance"`.

2. **Optimize Network Usage**: Avoid publishing data too frequently to minimize network traffic. Use conditional checks or rate limit publishers if necessary.

3. **Handle Data Latency**: Be aware of potential latency in data transmission. Implement timestamping if synchronization between data points is crucial.

4. **Security Considerations**: Ensure that sensitive data is not inadvertently published to unsecured Network Tables.

5. **Utilize WPILib Sendable Interfaces**: For more complex data structures or automatic dashboard integration, consider implementing WPILib's `Sendable` interfaces.

## Additional Resources

- **WPILib Network Tables Documentation**: [NetworkTables Overview](https://docs.wpilib.org/en/stable/docs/software/networktables/index.html)
- **WPILib Tutorials**: [NetworkTables Tutorial](https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-tutorial.html)
- **PhotonVision Documentation**: [PhotonVision Network Tables Integration](https://docs.photonvision.org/en/latest/networktables/)

---

By following this guide, you can effectively publish and manage telemetry data using Network Tables, enhancing your robot's monitoring capabilities and overall performance during competitions.
