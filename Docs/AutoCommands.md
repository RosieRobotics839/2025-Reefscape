# AutoCommands Documentation

The `AutoCommands` class provides utilities for automating robot actions during autonomous mode, making it easier to create complex sequences with minimal code. This document explains how `AutoCommands` works and how to use it effectively in your FRC robot code.

## Overview

`AutoCommands` is a utility class that contains static methods for common autonomous operations, such as:

- Getting coral game pieces from the source
- Scoring at the reef
- Navigating to specific field positions
- Storing and returning to poses

These methods create command sequences that can be chained together to form autonomous routines.

## Key Concepts

### AprilTag Navigation

Many commands leverage AprilTags (fiducial markers on the field) for precise positioning. The class includes helper methods that map AprilTag IDs to field locations for both alliance colors:

```java
// Helper functions for april tag selection on blue vs red alliance.
static public int coralSourceLTag(){ 
    return (DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get() == Alliance.Blue ? 13 : 1); 
}
static public int coralSourceRTag(){ 
    return (DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get() == Alliance.Blue ? 12 : 2); 
}
// More helper methods for other field elements...
```

This allows commands to work properly regardless of which alliance your robot is on.

## Main Commands

### 1. GetCoral Command

```java
public static Command GetCoral(boolean left)
```

This command navigates to the source (left or right side) and intakes a coral game piece. It:
1. Checks if the robot is outside the reef area
2. Moves the elevator down and arm up for intake
3. Navigates to a point near the source
4. Approaches the source
5. Runs the intake while performing a wiggle motion to help secure the game piece
6. Continues until a game piece is detected

Example usage:
```java
// Get coral from left source
Command getCoral = AutoCommands.GetCoral(true);
```

### 2. AutoScore Command

```java
public static Command AutoScore(String tag, boolean left, ScoreLevel level)
```

This complex command navigates to a reef and scores a game piece at the specified level. Parameters:
- `tag`: The target location on the reef (e.g., "NW", "NE", "SS")
- `left`: Whether to position on the left side of the tag
- `level`: The scoring level (FUNNEL, TROUGH, LEVEL2, LEVEL3, LEVEL4)

The command sequence:
1. Computes target positions for approach and final scoring
2. Navigates to the approach point
3. Starts raising the elevator and arm to the appropriate height
4. Approaches the final scoring position
5. Waits until in position and mechanisms are at target heights
6. Expels the game piece at the appropriate speed

Example usage:
```java
// Score at Level 4 on the northwest reef tag, from the left side
Command scoreCommand = AutoCommands.AutoScore("NW", true, ScoreLevel.LEVEL4);
```

### 3. Store and Return Commands

Two simple but powerful commands that work together:

```java
public static Command StorePose()
public static Command ReturnToPose()
```

These allow you to save the robot's current position and return to it later:

```java
// In autonomous sequence
autoSequence = Commands.sequence(
    AutoCommands.StorePose(),  // Store initial position
    AutoCommands.GetCoral(true),  // Get coral
    AutoCommands.ReturnToPose()  // Return to stored position
);
```

### 4. BargeFling Command

```java
public static Command BargeFling()
```

A specialized command for the algae game piece. It:
1. Raises the elevator to maximum height
2. Lowers the arm to minimum angle
3. Waits for mechanisms to reach position
4. Pauses briefly
5. Raises the arm quickly to "fling" the algae
6. Triggers the shooter when the arm approaches the top position

## Helper Commands

### Wiggle Command

An internal command that helps with intake:

```java
public static Command wiggle = Commands.sequence(
    Commands.waitSeconds(0.1),
    new InstantCommand(()->DriveTrain.getInstance().m_targetHeading += Units.degreesToRadians(10)),
    Commands.waitSeconds(0.1),
    new InstantCommand(()->DriveTrain.getInstance().m_targetHeading -= Units.degreesToRadians(20)),
    Commands.waitSeconds(0.1),
    new InstantCommand(()->DriveTrain.getInstance().m_targetHeading += Units.degreesToRadians(10))
).repeatedly().until(()->EndEffector.getInstance().hasGamePiece());
```

This oscillates the robot side-to-side while attempting to intake, which can help dislodge or better position game pieces.

### DriveReefOffset Method

```java
public static void DriveReefOffset(boolean left)
```

Computes and navigates to offset positions around the reef for scoring, supporting the AutoScore command.

## Tunable Parameters

Several parameters in `Constants.AutoConstants` can be tuned to optimize autonomous behavior:

| Parameter | Description | Tuning Advice |
|-----------|-------------|---------------|
| `kReefDistance` | Distance from reef for scoring | Decrease if not close enough to score |
| `kReefStartingDistance` | Initial approach distance | Increase for a more cautious approach |
| `kSourceDistance` | Distance from source for intake | Adjust based on intake mechanism reach |
| `kReefTolerance` | Position tolerance for scoring | Tighter tolerance = more precision but may take longer |
| `kTroughClearance` | Extra clearance for trough scoring | Increase if hitting the trough |
| `kReefOffset` | Side-to-side offset for scoring | Adjust based on robot's scoring mechanism position |
| `kLineupTimeout` | Maximum time for position alignment | Increase if positioning takes too long |

Example of a tunable parameter in the code:
```java
public static double kReefDistance = Units.inchesToMeters(NTDouble.create(6.25, 
    "Autonomous/kReefDistance", 
    val -> kReefDistance = Units.inchesToMeters(val)));
```

These parameters can be adjusted through NetworkTables in real-time, allowing for testing and tuning without code redeployment.

## Alliance-Aware Navigation

The `AutoCommands` class automatically handles alliance-specific navigation by using the correct AprilTag IDs and field positions based on the current alliance color. The `Autonomous` class provides helper methods like `reefCenter()` that return the appropriate position:

```java
public static Pose2d reefCenter(){ 
    return (DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get() == Alliance.Blue ? 
            Autonomous.m_blueReefCenter : Autonomous.m_redReefCenter); 
}
```

## Using AutoCommands in Autonomous Routines

To create a full autonomous routine, combine multiple commands in sequence:

```java
// Example 3-piece autonomous routine
Command myAutonomous = Commands.sequence(
    AutoCommands.GetCoral(true),                // Get first coral from left
    AutoCommands.AutoScore("NW", left, ScoreLevel.LEVEL4),  // Score high
    AutoCommands.GetCoral(false),               // Get second coral from right
    AutoCommands.AutoScore("NE", left, ScoreLevel.LEVEL4),  // Score high again
    AutoCommands.GetCoral(true),                // Get third coral from left
    AutoCommands.AutoScore("SW", left, ScoreLevel.LEVEL4)   // Score high once more
);
```

## Conclusion

`AutoCommands` provides a powerful, reusable framework for creating autonomous routines. By leveraging these commands, you can build complex autonomous sequences that adapt to the alliance color and field position while maintaining readable, maintainable code.

For advanced users, consider creating your own specialized commands by combining or extending the existing AutoCommands to fit your robot's specific capabilities and strategy requirements.
