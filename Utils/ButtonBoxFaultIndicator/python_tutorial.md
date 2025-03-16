# Understanding Python Programs for FRC Students

## Introduction

Welcome to your first Python project! If you've been programming FRC robots in Java, you'll find many similarities in Python. This guide will help you understand the anatomy of a Python program by drawing parallels to the Java robot code you're already familiar with.

## Python vs. Java: The Basics

| Concept | Java | Python |
|---------|------|--------|
| End of statements | Semicolons `;` | Newlines (no semicolons needed) |
| Code blocks | Curly braces `{ }` | Indentation (typically 4 spaces) |
| Classes | `public class Robot { }` | `class Robot:` |
| Methods | `public void teleopPeriodic() { }` | `def teleop_periodic():` |
| Variables | `double motorSpeed = 0.5;` | `motor_speed = 0.5` |
| Constants | `final double kP = 0.1;` | `kP = 0.1` |

## Anatomy of Your Python NetworkTables Program

Let's look at the structure of your Python program and compare it to your robot code:

### 1. Imports

```python
# Python imports (like Java imports)
from networktables import NetworkTables
import serial
import time
import json
```

This is similar to the imports in your Java code:
```java
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
// Other imports...
```

### 2. Constants and Configuration

```python
# Constants (similar to Constants.java)
ROBOT_IP = "10.XX.XX.2"
ARDUINO_PORT = "COM3"
BAUD_RATE = 9600
```

This is similar to your Java constants:
```java
// From Constants.java
public static final class ArmConstants {
    public static final int kCANID = 11;
    // Other constants...
}
```

### 3. Functions (Like Methods in Java)

```python
def connect_to_networktables(ip):
    """Connect to the NetworkTables server on the robot"""
    NetworkTables.initialize(server=ip)
    # Rest of function...
```

This is similar to methods in your Java subsystems:
```java
public double getArmPosition(){
    return m_motor.getPosition() * (2 * Math.PI);
}
```

### 4. Main Function and Program Entry Point

```python
def main():
    """Main program loop"""
    # Main loop code here...

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program terminated by user")
```

This is similar to the periodic methods in your robot code:
```java
@Override
public void periodic() {
    // Periodic code...
}
```

The `if __name__ == "__main__":` line is Python's way of saying "run this code when this file is executed directly" (not imported by another file).

## NetworkTables in Python vs. Java

### Java (from your robot code):
```java
// Creating NetworkTable
static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Arm/table");

// Publishing values
DoublePublisher nt_currentAngle = table.getDoubleTopic("angle/currentAngle").publish();
nt_currentAngle.set(Units.radiansToDegrees(getArmPosition()));
```

### Python (in your new program):
```python
# Getting a table
drivetrain = NetworkTables.getTable("roboRIO/Drivetrain")

# Reading values
left_speed = drivetrain.getNumber("leftSpeed", 0.0)
```

## Key Differences to Remember

1. **Indentation Matters**: In Python, indentation defines code blocks (instead of Java's curly braces)

2. **No Type Declarations**: Python variables don't need types declared:
   - Java: `double motorSpeed = 0.5;`
   - Python: `motor_speed = 0.5`

3. **Function Definitions**: Use `def` instead of method declarations:
   - Java: `public void setPosition(double radians) { }`
   - Python: `def set_position(radians):`

4. **Snake Case**: Python conventionally uses snake_case for functions and variables (not camelCase like Java)

## Reading Your Robot's NetworkTables

Looking at your Java code, your arm publishes these values to NetworkTables:
- `roboRIO/Arm/table/angle/currentAngle`
- `roboRIO/Arm/table/angle/targetAngle`
- `roboRIO/Arm/table/angle/positionSensor` 
- And more...

In your Python program, you'll read these values with:
```python
arm_table = NetworkTables.getTable("roboRIO/Arm/table")
current_angle = arm_table.getNumber("angle/currentAngle", 0.0)
```

Remember: Python is more relaxed about syntax than Java, but be consistent with your indentation and naming style!

Happy coding!
