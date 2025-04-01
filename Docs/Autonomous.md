# Autonomous Subsystem Documentation

## Overview

The Autonomous subsystem in our robot code manages autonomous navigation, obstacle avoidance, and target aiming. It integrates with the PathPlanning system to navigate around the field while avoiding static obstacles like the reef.

The subsystem provides several key functionalities:
- Reef obstacle detection and avoidance
- Automatic aiming at targets (like scoring locations)
- Static obstacle definition and management
- Helper methods for autonomous scoring and game piece collection

## Key Components

### Obstacle Definition

The Autonomous subsystem defines static obstacles on the field that the robot needs to avoid:

```java
public static List<Translation2d> generateReefKOZ(Pose2d center){
  return new ArrayList<Translation2d>(){{
    for (int i=0; i<6; i++){
      add(center.getTranslation().plus(
        new Translation2d(
          AutoConstants.kReefKOZRadius,
          new Rotation2d(center.getRotation().getRadians()+Units.degreesToRadians(30+60*i))
        )
      ));
    }
  }};
}
```

This code creates a hexagonal "Keep Out Zone" (KOZ) around the reef in the center of the field. The reef is a critical game element that the robot must navigate around, not over.

### Aiming System

The Autonomous subsystem can aim the robot at specific points on the field, which is especially useful for lining up to score:

```java
public void aimAtPoint(Pose2d aimPoint, double radiansOffset){
  m_aimPoint = aimPoint;
  m_aimPointRotationOffset = radiansOffset;
}
```

When an aim point is set, the DriveTrain subsystem will automatically try to orient the robot toward this point during movement.

### Reef Detection

The subsystem detects when the robot is inside the reef area:

```java
nt_isInsideReef.set(m_insideReefDebounce.calculate(
  PathfindingUtils.PointInConvexPolygon(
    PoseEstimator.getInstance().m_finalPose.getTranslation(), 
    reefObstacle()
  )
));
```

This uses a debouncer to prevent rapid toggling and the `PathfindingUtils` helper to determine if the robot's position is inside the defined reef polygon.

## Helper Utilities

### PathfindingUtils

The `PathfindingUtils` class provides several methods for obstacle detection and polygon intersection:

```java
public static boolean PointInConvexPolygon(Translation2d point, List<Translation2d> polygon){
  for (int i = 0; i < polygon.size(); i++) {
    Translation2d p1 = polygon.get(i);
    Translation2d p2 = polygon.get((i + 1) % polygon.size());
    if (crossProduct(p1, p2, point) < 0) {
      return false; // Point is on the wrong side of an edge
    }
  }
  return true;
}
```

This uses a mathematical approach to determine if a point is inside a convex polygon, which is faster than other point-in-polygon algorithms for convex shapes.

### VectorUtils

The `VectorUtils` class provides vector operations that help with position calculations and path planning:

```java
public static Pose2d poseDiff(Pose2d pose1, Pose2d pose2){
  Rotation2d rotation;
  if (pose1.getRotation() != null && pose2.getRotation() != null){
    rotation = new Rotation2d(angleDifference(
          pose1.getRotation().getRadians(),
          pose2.getRotation().getRadians()
    ));
  } else {
    rotation = new Rotation2d(0);
  }

  return new Pose2d(pose1.getX()-pose2.getX(), pose1.getY()-pose2.getY(), rotation);
}
```

## Tunable Parameters

The Autonomous subsystem has several important tunable parameters:

### Reef KOZ Radius

```java
public static double kReefKOZRadius = Units.feetToMeters(NTDouble.create(6,"Autonomous/kReefKOZRadius", val -> {
  kReefKOZRadius = Units.feetToMeters(val);
  Autonomous.staticObstacles = Autonomous.generateStaticObstacles();
}));
```

This defines how large the "keep out zone" around the reef should be. Increasing this value makes the robot stay further away from the reef, which is safer but may make paths longer.

**How to tune**: Start with the default value (6 feet). If the robot is getting too close to the reef during autonomous, increase this value. If the robot is taking unnecessarily long paths around the reef, decrease it slightly.

### Lineup Timeout

```java
public static double kLineupTimeout = NTDouble.create(7, "Autonomous/kLineupTimeout", val -> kLineupTimeout = val);
```

This defines how long the robot will attempt to line up for scoring before timing out.

**How to tune**: If the robot is taking too long to line up for scoring, decrease this value. If the robot isn't getting enough time to line up properly, increase it.

### Reef Distance Parameters

```java
public static double kReefDistance = Units.inchesToMeters(NTDouble.create(6.25, "Autonomous/kReefDistance", val -> kReefDistance = Units.inchesToMeters(val)));
public static double kReefStartingDistance = Units.inchesToMeters(NTDouble.create(36.0, "Autonomous/kReefStartingDistance", val -> kReefStartingDistance = Units.inchesToMeters(val)));
public static double kReefTolerance = Units.inchesToMeters(NTDouble.create(0.5, "Autonomous/kReefToleranceInch", val -> kReefTolerance = Units.inchesToMeters(val)));
```

These parameters control how close the robot gets to the reef during scoring operations:
- `kReefDistance`: Final stopping distance from the reef
- `kReefStartingDistance`: Initial approach distance
- `kReefTolerance`: How close the robot needs to get to its target position

**How to tune**: 
- Increase `kReefDistance` if the robot is bumping into the reef
- Decrease `kReefDistance` if the robot is stopping too far away to score effectively
- Adjust `kReefTolerance` based on how precisely the robot needs to position (smaller values require more precision)

### Reef Offset

```java
public static double kReefOffset = Units.inchesToMeters(NTDouble.create(6.5,"Autonomous/kReefOffset",val -> kReefOffset = Units.inchesToMeters(val)));
public static double kStaticReefOffset = Units.inchesToMeters(NTDouble.create(4,"Autonomous/kStaticReefOffset",val -> kStaticReefOffset = Units.inchesToMeters(val)));
```

These parameters adjust the lateral offset when approaching the reef for scoring:
- `kReefOffset`: Dynamic offset that can be positive (left) or negative (right)
- `kStaticReefOffset`: A constant offset added to all approaches

**How to tune**: If the robot is consistently scoring off-center, adjust these values to shift the approach position.

## Important Methods

### AutoScore

The `AutoScore` method is used to approach a reef tag and score a game piece:

```java
public static Command AutoScore(String tag, boolean left, ScoreLevel level){
  // ... code to determine which tag to target ...
  
  Pose2d target1 = PathPlanning.AprilTagAtDistance(
    tagId,
    new Translation2d(
      - AutoConstants.kReefStartingDistance - Constants.kChassis.kWheelBase/2.0,
      Constants.AutoConstants.kReefOffset * (left ? 1 : -1) + Constants.AutoConstants.kStaticReefOffset
    ),Units.degreesToRadians(15)
  );

  // ... more code ...
  
  return Commands.sequence(
    // ... sequence of commands to navigate and score ...
  );
}
```

### GetCoral

The `GetCoral` method handles autonomously retrieving a game piece from a source:

```java
public static Command GetCoral(int tagId){
  return Commands.parallel(
    // ... commands to move elevator and arm ...
    Commands.sequence(
      // ... navigation commands ...
      Commands.parallel(
        EndEffector.getInstance().IntakeCommand().asProxy(),
        wiggle.asProxy()
      ).withTimeout(5)
    ).repeatedly().handleInterrupt(()->DriveTrain.getInstance().m_poseQueue.clear())
  ).until(()->{return !Autonomous.getInstance().isInsideReef() && EndEffector.getInstance().hasGamePiece();});
}
```

## Usage Example

Here's an example of how you might use the Autonomous subsystem in an autonomous routine:

```java
// Create an autonomous routine to get coral and then score it
Command autonomousRoutine = Commands.sequence(
  // Drive to and collect coral from the right source
  AutoCommands.GetCoral(false),
  
  // Score the coral at the northwest reef position, left side, level 4
  AutoCommands.AutoScore("NW", true, ScoreLevel.LEVEL4)
);
```

## Advanced Concepts

### A* Pathfinding Integration

The Autonomous subsystem works closely with the PathPlanning subsystem, which uses an A* pathfinding algorithm to find routes around obstacles. The obstacles defined in Autonomous (like the reef) are used by PathPlanning to create paths that avoid these areas.

### Convex Polygon Math

The system uses mathematical concepts like cross products to determine if points are inside polygons:

```java
private static double crossProduct(Translation2d p1, Translation2d p2, Translation2d point) {
  return (p2.getX() - p1.getX()) * (point.getY() - p1.getY()) - 
         (point.getX() - p1.getX()) * (p2.getY() - p1.getY());
}
```

This uses the cross product of vectors to determine if a point is on the "inside" or "outside" of an edge of the polygon. For a convex polygon, if the point is on the correct side of all edges, it's inside the polygon.

## Troubleshooting

If the robot is having trouble navigating autonomously:

1. Check if the reef KOZ is appropriately sized - too large can cause unnecessary detours
2. Verify the Vision system is providing accurate pose estimates
3. Check distance parameters if the robot is stopping too far from or too close to targets
4. Use the "AutoTraj" field object visualization in Glass to see the planned paths

Understanding and tuning these parameters will help you maximize the performance of the autonomous navigation system and ensure reliable scoring during autonomous periods.
