# PathPlanning System Documentation

## Overview

The PathPlanning system enables our robot to navigate autonomously across the field, finding efficient routes while avoiding obstacles. This document explains how the system works, its key components, and how to use it effectively.

## Key Components

Our autonomous navigation system consists of several key components:

1. **PathPlanning Class**: The main class that manages navigation and pathfinding.
2. **A* Algorithm**: An efficient pathfinding algorithm implemented through several classes.
3. **Field Graph**: A network of nodes representing navigable locations on the field.
4. **Obstacle Avoidance**: Logic to detect and route around obstacles like the reef.

## How PathPlanning Works

### Basic Navigation

To command the robot to drive to a specific point on the field, we use the `navigateTo` method:

```java
// Navigate to a position
PathPlanning.getInstance().navigateTo(targetPose);
```

This adds the target position to a queue in the DriveTrain subsystem, which then manages the driving to that position.

### Path Generation Process

When you call `navigateTo()`, several things happen:

1. The system checks if the starting point is inside an obstacle
2. If inside an obstacle, it first plans a path to exit the obstacle
3. It uses A* search to find an optimal path through navigable areas
4. The path is converted into a series of waypoints
5. These waypoints are added to the drive queue for execution

### Field Graph Creation

The field is represented as a graph of connected nodes. For our game (Reefscape), we create nodes around the reef to navigate around it:

```java
public void GenerateReefNodes() {
    for (Integer i=0; i<6; i++) {
        addNode(new FieldPose(
            i,
            i.toString(),
            new Pose2d(
                Autonomous.reefCenter().getTranslation().plus(
                    new Translation2d(
                        AutoConstants.kReefGraphNodeRadius,
                        new Rotation2d(Autonomous.reefCenter().getRotation().getRadians()+
                            Units.degreesToRadians(30+60*i))
                    )
                ),
                null
            )
        ),
        // Connections between nodes - forms a hexagon around the reef
        new int[]{(i+5) % 6, (i+7) % 6}
        );
    };
}
```

This creates a hexagonal pattern of nodes around the reef, with each node connecting to its neighbors.

## A* Pathfinding Implementation

### Core Components

1. **Graph**: Represents the field with nodes and connections
2. **FieldPose**: Represents a position on the field
3. **Scorer**: Determines the cost of moving between nodes
4. **RouteFinder**: The A* implementation that finds the optimal path

### How A* Works

The A* algorithm works by:

1. Starting at the current position
2. Exploring possible paths, prioritizing those that seem closest to the goal
3. Calculating costs for each path segment based on distance and obstacles
4. Finding the lowest-cost path to the destination

```java
// Finding a route using A*
RouteFinder<FieldPose> routeFinder = new RouteFinder<FieldPose>(
    fieldgraph,  // The graph of the field
    nextNodeScorer,  // Determines cost between adjacent nodes
    targetScorer     // Estimates cost to reach goal
);
List<FieldPose> route = routeFinder.findRoute(m_current, m_destination);
```

### Scoring System

The scoring system has two components:

1. **NextNodeScorer**: Evaluates the real cost of moving between adjacent nodes
   - Considers obstacles, distances, and penalties
   - Heavily penalizes paths through restricted areas

2. **TargetScorer**: Provides a heuristic estimate of the distance to the goal
   - Uses straight-line distance as an optimistic estimate

```java
// Example of how the PathScorer evaluates cost between nodes
public double computeCost(FieldPose from, FieldPose to) {
    // Base cost is the direct distance between points
    double cost = VectorUtils.poseDiff(to.getPose(), from.getPose())
                  .getTranslation().getNorm();
    
    // Add enormous penalty if path intersects an obstacle
    if (PathfindingUtils.PathIntersectsPolygon(
        from.getPose().getTranslation(),
        to.getPose().getTranslation(),
        Autonomous.staticObstacles)) {
        cost += 999999;
    }
    
    return cost;
}
```

## Helper Utilities

### Position and Heading Utilities

The system includes utilities to help define target positions:

```java
// Get position at specific distance from AprilTag
Pose2d targetPos = PathPlanning.AprilTagAtDistance(
    tagId,  // AprilTag ID
    -2.0,   // 2 meters away from tag
    Math.PI // Facing the tag
);

// Navigate to that position
PathPlanning.getInstance().navigateTo(targetPos);
```

### VectorUtils

The `VectorUtils` class provides important geometry functions like:

- `isNear`: Checks if a position is close to a target
- `poseDiff`: Calculates the difference between poses
- `angleDifference`: Finds the smallest angle between two headings
- `nearestPointOnLine`: Finds the closest point on a line segment
- `findIntersection`: Determines if and where two line segments intersect

```java
// Check if we're near our target position
if (VectorUtils.isNear(
    PoseEstimator.getInstance().m_finalPose,
    targetPose,
    0.1,  // Position tolerance (meters)
    0.05  // Rotation tolerance (radians)
)) {
    // We've reached our target
}
```

### Obstacle Detection

The `PathfindingUtils` class helps with obstacle detection:

```java
// Check if a point is inside an obstacle
boolean isInObstacle = PathfindingUtils.PointInConvexPolygon(
    robotPosition,
    obstacleVertices
);

// Check if a path intersects an obstacle
boolean pathBlocked = PathfindingUtils.PathIntersectsPolygon(
    startPoint,
    endPoint,
    listOfObstacles
);
```

## Advanced Features

### Partial Navigation

Sometimes we want to navigate close to a target but not directly to it:

```java
// Navigate to within 0.5 meters of a target
PathPlanning.getInstance().navigateCloseTo(
    targetPose,       // Target position
    0.5,              // Distance to maintain (meters)
    Math.PI/4,        // Acceptable angle range (radians)
    false             // Stop if already within range
);
```

### AprilTag-based Navigation

A key feature is navigating relative to AprilTags:

```java
// Navigate to position 2 meters in front of AprilTag #7
Pose2d target = PathPlanning.AprilTagAtDistance(
    7,        // AprilTag ID
    -2.0,     // Distance from tag (negative = in front)
    Math.PI   // Facing the tag
);
PathPlanning.getInstance().navigateTo(target);
```

## Understanding the Code

### Pose Representation

- `Pose2d`: Represents position (x,y) and rotation on the field
- `Translation2d`: Represents a 2D vector (x,y) without rotation
- `Rotation2d`: Represents an angle

### Route Finding

The core of the A* implementation is in `RouteFinder.findRoute()`:

```java
public List<T> findRoute(T from, T to) {
    // Initialize route nodes with infinite scores
    List<RouteNode<T>> allNodes = graph.nodes.stream()
        .map(n -> new RouteNode<T>(n))
        .collect(Collectors.toList());
    
    // Priority queue of nodes to explore
    TreeSet<RouteNode<T>> openSet = new TreeSet<>();
    
    // Start with the "from" node, setting score to 0
    RouteNode<T> start = allNodes.get(from.getId());
    start.setRouteScore(0);
    openSet.add(start);
    
    while (!openSet.isEmpty()) {
        // Get the node with the lowest score
        RouteNode<T> next = openSet.pollFirst();
        
        // If we've reached our destination, build and return the path
        if (next.getCurrent().equals(to) && next.getRouteScore() < 50000) {
            List<T> route = new ArrayList<>();
            RouteNode<T> current = next;
            do {
                route.add(0, current.getCurrent());
                if(current.getPrevious() == null) break;
                current = allNodes.get(current.getPrevious().getId());
            } while (current != null);
            return route;
        }
        
        // Check all connected nodes
        graph.getConnections(next.getCurrent()).forEach(connection -> {
            // Calculate score for this path
            double newScore = next.getRouteScore() + 
                nextNodeScorer.computeCost(next.getCurrent(), connection);
                
            RouteNode<T> nextNode = allNodes.get(connection.getId());
            // If we found a better path to this node
            if (nextNode != null && nextNode.getRouteScore() > newScore) {
                nextNode.setPrevious(next.getCurrent());
                nextNode.setRouteScore(newScore);
                nextNode.setEstimatedScore(newScore + 
                    targetScorer.computeCost(connection, to));
                openSet.add(nextNode);
            }
        });
    }
    
    // No route found
    return new ArrayList<>();
}
```

## Using the PathPlanning System

### Basic Usage

```java
// Navigate to a specific position
PathPlanning.getInstance().navigateTo(new Pose2d(5.0, 3.0, new Rotation2d(0)));

// Navigate to an AprilTag
PathPlanning.getInstance().navigateTo(
    PathPlanning.AprilTagAtDistance(7, -1.0)
);
```

### Advanced Usage

```java
// Clear the navigation queue
DriveTrain.getInstance().m_poseQueue.clear();

// Plan a multi-segment path
Pose2d waypoint1 = new Pose2d(2.0, 3.0, new Rotation2d(0));
Pose2d waypoint2 = new Pose2d(4.0, 3.5, new Rotation2d(Math.PI/4));
Pose2d target = new Pose2d(6.0, 4.0, new Rotation2d(Math.PI/2));

// Sequential navigation
PathPlanning.getInstance().navigateTo(waypoint1);
PathPlanning.getInstance().navigateTo(waypoint2);
PathPlanning.getInstance().navigateTo(target);
```

## Conclusion

The PathPlanning system provides a robust way for our robot to navigate autonomously. By understanding the graph representation, A* pathfinding, and helper utilities, you can effectively plan autonomous paths for the robot to follow.

Remember that the PathPlanning system relies on accurate pose estimation from the Vision and Gyro subsystems. Always make sure these are properly calibrated for the best results.
