# PID Controller Documentation

## Overview

The `PIDController` class is a fundamental control system utility used in our FRC robot to maintain precise control of various mechanisms. This controller uses three control terms—Proportional, Integral, and Derivative—to calculate an output that brings the robot's mechanisms to a desired setpoint.

## How PID Works

A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable. It applies a correction based on proportional, integral, and derivative terms.

![PID Controller Diagram](https://upload.wikimedia.org/wikipedia/commons/4/43/PID_en.svg)

### Components of PID

```
Output = Kp * error + Ki * ∫error·dt + Kd * d(error)/dt
```

Where:
- **P (Proportional)**: Responds to present error
- **I (Integral)**: Responds to accumulated past error
- **D (Derivative)**: Predicts future error

## PIDController Class Structure

Our `PIDController` class extends the WPILib PID controller with additional features:

```java
public class PIDController implements Sendable, AutoCloseable {
    // Controller coefficients
    private double m_kp;
    private double m_ki;
    private double m_kd;
    
    // Time period between calculations
    private final double m_period;
    
    // Error values
    private double m_positionError;
    private double m_velocityError;
    private double m_prevError;
    private double m_totalError;
    
    // Other controller parameters
    private double m_maximumIntegral = 1.0;
    private double m_minimumIntegral = -1.0;
    private double m_positionTolerance = 0.05;
    private double m_velocityTolerance = Double.POSITIVE_INFINITY;
    
    // Input/output values
    private double m_setpoint;
    private double m_measurement;
    
    // State flags
    private boolean m_haveMeasurement;
    private boolean m_haveSetpoint;
    private boolean m_continuous;
    private double m_maximumInput;
    private double m_minimumInput;
}
```

## Key Methods

### Constructor

```java
public PIDController(double kp, double ki, double kd, double period) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_period = period;
    // ...
}
```

### Setting PID Coefficients

```java
public void setPID(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}
```

### Calculate Method

The heart of the PID controller is the `calculate()` method, which computes the next output value:

```java
public double calculate(double measurement) {
    m_measurement = measurement;
    m_prevError = m_positionError;
    m_haveMeasurement = true;

    // Calculate position error (accounts for continuous inputs like angles)
    if (m_continuous) {
        double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
        m_positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
        m_positionError = m_setpoint - m_measurement;
    }

    // Calculate velocity error
    m_velocityError = (m_positionError - m_prevError) / m_period;

    // Update integral term with anti-windup protection
    if (m_ki != 0) {
        m_totalError = MathUtil.clamp(
            m_totalError + m_positionError * m_period,
            m_minimumIntegral / m_ki,
            m_maximumIntegral / m_ki);
    }

    // Calculate final output
    double output = m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
    
    // Network table debug output
    if (table != null){
        nt_fb.set(m_ntscale*m_measurement);
        nt_err.set(m_ntscale*m_positionError);
        nt_verr.set(m_ntscale*m_velocityError);
        nt_output.set(output);
        nt_p.set(m_kp*m_positionError);
        nt_i.set(m_ki*m_totalError);
        nt_d.set(m_kd*m_velocityError);
    }

    return output;
}
```

### Setpoint and Tolerance Methods

```java
// Set the desired target value
public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
    m_haveSetpoint = true;
    // Calculate errors based on new setpoint
}

// Define how close is "close enough" to target
public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
}

// Check if we've reached the target
public boolean atSetpoint() {
    return m_haveMeasurement
        && m_haveSetpoint
        && Math.abs(m_positionError) < m_positionTolerance
        && Math.abs(m_velocityError) < m_velocityTolerance;
}
```

## Network Table Integration

Our `PIDController` includes comprehensive network table integration for debugging and tuning:

```java
public void publish(NetworkTable _table, String topic){
    table = _table;
    nt_err = table.getDoubleTopic(topic+"/Error/Position").publish();
    nt_verr = table.getDoubleTopic(topic+"/Error/Velocity").publish();
    nt_set = table.getDoubleTopic(topic+"/1_Setpoint").publish();
    nt_fb = table.getDoubleTopic(topic+"/2_Feedback").publish();
    nt_output = table.getDoubleTopic(topic+"/3_Output").publish();
    nt_p = table.getDoubleTopic(topic+"/_Output/P").publish();
    nt_i = table.getDoubleTopic(topic+"/_Output/I").publish();
    nt_d = table.getDoubleTopic(topic+"/_Output/D").publish();
    nt_imax = table.getDoubleTopic(topic+"/Limits/Imax").publish();
    nt_imin = table.getDoubleTopic(topic+"/Limits/Imin").publish();

    nt_kp = new NTDouble(m_kp,table,topic+"/Kp",val->m_kp=val);
    nt_ki = new NTDouble(m_ki,table,topic+"/Ki",val->m_ki=val);
    nt_kd = new NTDouble(m_kd,table,topic+"/Kd",val->m_kd=val);

    nt_scale = new NTDouble(1.0,table,topic+"/NTScale",val->m_ntscale=val);

    nt_tol_p = table.getDoubleTopic(topic+"/Tolerance/Pos").publish();
    nt_tol_v = table.getDoubleTopic(topic+"/Tolerance/Vel").publish();
}
```

This allows:
1. Real-time monitoring of all controller values
2. Live tuning of PID gains through network tables
3. Visualization of controller performance

## Special Features

### Continuous Input

For mechanisms like angle controllers where the input wraps around (e.g., 359° is close to 0°), we can enable continuous input:

```java
// Enable continuous input for mechanisms like heading control
controller.enableContinuousInput(0, 2*Math.PI);  // For angles in radians
```

### Integrator Range Control

To prevent integral windup (where the I term grows too large):

```java
// Set limits on how large the integral term can get
controller.setIntegratorRange(-0.5, 0.5);
```

### Access to Individual Terms

You can access individual PID terms for advanced debugging:

```java
double pEffort = controller.getEffortP();
double iEffort = controller.getEffortI();
double dEffort = controller.getEffortD();
```

## Tuning a PID Controller

Tuning a PID controller is part science, part art. Here's a step-by-step approach:

1. **Start with all gains at zero**: Kp = 0, Ki = 0, Kd = 0

2. **Increase Kp**: 
   - Start by increasing proportional gain (Kp) until the system responds quickly but may oscillate around the setpoint
   - If the system oscillates too much or overshoots, reduce Kp slightly

3. **Add Kd**:
   - Add derivative gain (Kd) to dampen oscillations
   - Start small and increase until oscillations are reduced
   - Too much Kd will make the system sluggish or jittery

4. **Finally, add Ki**:
   - Add integral gain (Ki) to eliminate steady-state error
   - Start very small (often 1/10 of Kp or less)
   - Increase until any persistent error is eliminated
   - Too much Ki will cause slow oscillations

5. **Fine-tune**:
   - Make small adjustments to all three gains to optimize performance
   - Use the network table values to monitor performance

### Common Issues in PID Tuning

| Problem | Symptoms | Solution |
|---------|----------|----------|
| Oscillations | System constantly overshoots setpoint | Decrease Kp, increase Kd |
| Sluggish response | System moves slowly to setpoint | Increase Kp |
| Steady-state error | System gets close but not exactly to setpoint | Increase Ki |
| Overshoot | System goes past setpoint before settling | Increase Kd, decrease Kp |
| Jittering | System vibrates or is noisy | Decrease Kd, add filtering |

## Example Usage

Here's a complete example of using our PID controller for robot heading control:

```java
// Create PID controller with gains and period of 0.02 seconds (50Hz)
PIDController m_headingPID = new PIDController(2.0, 0, 0.05, 0.02);

// Configure for continuous heading (0-360°)
m_headingPID.enableContinuousInput(0, Math.PI*2);

// Set tolerance for "close enough" to target
m_headingPID.setTolerance(Math.toRadians(2), Math.toRadians(5));

// Make it visible on network tables for tuning
m_headingPID.publish(NetworkTableInstance.getDefault().getTable("Drivetrain"), "HeadingPID");

// Use in periodic function
public void periodic() {
    // Get current angle
    double currentHeading = gyro.getYaw();
    
    // Calculate rotation command (limited to +/- max rotation speed)
    double rotateCommand = MathUtil.clamp(
        m_headingPID.calculate(currentHeading, targetHeading),
        -maxRotationSpeed,
        maxRotationSpeed
    );
    
    // Apply to drive system
    drivetrain.drive(forwardSpeed, sidewaysSpeed, rotateCommand);
}
```

## Advanced Concepts

### Anti-Windup Protection

Our PID controller includes anti-windup protection that prevents the integral term from growing out of control:

```java
m_totalError = MathUtil.clamp(
    m_totalError + m_positionError * m_period,
    m_minimumIntegral / m_ki,
    m_maximumIntegral / m_ki);
```

### Continuous Input Handling

For angular measurements, the controller automatically handles angle wrapping:

```java
if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError = MathUtil.inputModulus(
        m_setpoint - m_measurement, 
        -errorBound, 
        errorBound
    );
}
```

This ensures that if your setpoint is 350° and your current position is 10°, the error is calculated as -20° (shortest path) rather than 340°.

## Conclusion

The `PIDController` class is a powerful tool for precise control of robot mechanisms. By understanding how to create, configure, and tune PID controllers, you can achieve much more precise and reliable robot behavior across all subsystems.

Remember that tuning is iterative - start simple, make small changes, and use the network table data to guide your tuning process!
