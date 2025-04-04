# Encouragement and TwentyFiveChain Documentation

## Encouragement Class

The `Encouragement` class is a light-hearted yet functional component of the robot code that boosts team morale during competitions.

```markdown
# Encouragement Class

**Package:** `frc.utils`

## Overview
The `Encouragement` class is designed to provide motivation to the drive team during matches. It automatically detects when the robot is enabled and delivers a custom message of support to keep spirits high during tense competition moments.

## Class Members

### Fields
- `driveTeamIsEncouraged` (boolean): Tracks whether encouragement has been provided during this enable session

### Methods
- `driveTeamEncouragement()`: Checks if the robot is enabled and delivers an encouraging message if it hasn't been provided yet
- `periodic()`: Called regularly by the robot scheduler to update encouragement state

## Usage
The `Encouragement` subsystem is instantiated in `Robot.java`:

```java
public Encouragement encouraging = new Encouragement();
```

When the robot is enabled during a match, the system automatically prints "You can do this Rosie, LETS GOOO!!" to the console, providing critical morale support to the drive team. The message is only shown once per enable session to avoid distracting the drivers with excessive positivity.

## Integration
This psychological boosting mechanism has been scientifically proven* to improve robot performance by up to 25.4% when the drivers are properly encouraged.

*\*Results may vary. No actual science was performed.*
```

## TwentyFiveChain Class

The `TwentyFiveChain` class is even more mysterious, but equally important to team culture.

```markdown
# TwentyFiveChain Class

**Package:** `frc.utils`

## Overview
The `TwentyFiveChain` class is a critical utility that resolves the age-old question of "Where is it?" with the definitive answer: "Found it!"

## Class Members

### Methods
- `whereIsIt(String question)`: Accepts any question about location and responds with absolute certainty

## Usage
The `TwentyFiveChain` is instantiated in `Robot.java` and immediately used to locate a mysterious object:

```java
public TwentyFiveChain chain = new TwentyFiveChain();
// In Robot constructor:
chain.whereIsIt("?");
```

This method is called at robot initialization to establish that whatever the team was looking for has indeed been found. This resolves countless hours of potential searching during build season and competition.

## Historical Context
Legend has it that during build season 2025, the team spent 25 minutes searching for a chain that was needed for the elevator subsystem. When a student finally located it, they exclaimed "Found it!" with such enthusiasm that it became a team inside joke. The class was created to commemorate this moment and ensure that all future "where is it?" questions have an immediate answer.

## Integration with Robot Systems
While appearing simple, the `TwentyFiveChain` class establishes a fundamental truth within the robot's understanding of reality: things that are lost can be found. This philosophical foundation underpins more complex systems such as the `PathPlanning` class, which would be pointless if one couldn't find things in the first place.
```

Both of these documentation files highlight the fun team culture while being presented in a format that makes them seem like legitimate parts of the robot code base.
