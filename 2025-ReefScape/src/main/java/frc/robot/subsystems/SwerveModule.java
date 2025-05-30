// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.robot.Constants.kDriveTrain.kSwerveModule;
import frc.robot.Robot;
import frc.utils.FirstOrderLag;
import frc.utils.Motor;
import frc.utils.NTValues.NTDouble;
import frc.robot.Constants.CANID_t;

public class SwerveModule extends SubsystemBase {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain/wheel");
  static NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/Test");

  public Motor m_motorDrive, m_motorSteer;

  int m_steeringOffset;
  SwerveModuleState optimizedState = new SwerveModuleState(0, new Rotation2d(0));

  public boolean m_setupDriveDone = false;
  public boolean m_setupSteerDone = false;

  public AnalogInput m_analogEncoder;
  public double angleCalibration;
  public FirstOrderLag m_magLimiter = new FirstOrderLag(DriveConstants.kLinearAccelerationTau, 0, 0.020);

  DoublePublisher 
    nt_angleinit,
    nt_angle,
    nt_speedcmd,
    nt_anglecmd,
    nt_speed, nt_i, nt_analog, nt_steeri, nt_steeringOffset;

  double testAnglePerSec, testAngle, testSpeed;

  public SwerveModule(CANID_t CANID, double angleCalibration, String name) {
   
    NTDouble.create(0,testtable,"SwerveModule/"+name+"/degpersec",val->this.testAnglePerSec=val);
    NTDouble.create(0,testtable,"SwerveModule/"+name+"/feetpersec",val->this.testSpeed=val);

    // Setup Network Table Publishers
    nt_angleinit = table.getDoubleTopic("angle/init/"+name).publish();
    nt_angle = table.getDoubleTopic("angle/"+name).publish();
    nt_speed = table.getDoubleTopic("speed/"+name).publish();
    nt_speedcmd = table.getDoubleTopic("speedcmd/"+name).publish();
    nt_anglecmd = table.getDoubleTopic("anglecmd/"+name).publish();
    nt_i = table.getDoubleTopic("current/"+name).publish();
    nt_steeri = table.getDoubleTopic("current/"+name).publish();
    nt_analog = table.getDoubleTopic("analog/"+name).publish();
    nt_steeringOffset = table.getDoubleTopic("angle/steeringOffset/"+name).publish();

    /* Define drive motor controller. */
    m_motorDrive = new Motor(CANID.driving, kSwerveModule.kDriveType, name+"_driving")
        .inverted(false)
        .idleBrake(true)
        .withStatorLimit((int)kSwerveModule.kDrivingMotorCurrentLimit)
        .withSlowSpeedControl((Robot.isSimulation() ? false : true))
        .withGearRatio(kSwerveModule.kDriveMotorGearReduction)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kSwerveModule.kDriveKp, kSwerveModule.kDriveKi, kSwerveModule.kDriveKd, kSwerveModule.kDriveKff, Motor.GainSlot.SPEED)
        .pidf(kSwerveModule.kDrivePosKp, 0, 0, 0, Motor.GainSlot.POSITION)
        .withOutputRange(-1,1)
        .withIZone(0.15); 

    /* Define steer analog encoder and store calibration value */
    m_analogEncoder = new AnalogInput(CANID.encoder);
    this.angleCalibration = angleCalibration;
    m_steeringOffset = m_analogEncoder.getValue();
    nt_steeringOffset.set(m_steeringOffset);
    
    /* Define steer motor controller. */
    m_motorSteer = new Motor(CANID.steering, kSwerveModule.kSteerType, name+"_steering")
      .inverted(true)
      .idleBrake(true)
      .withStatorLimit((int)kSwerveModule.kSteeringMotorCurrentLimit)
      .withGearRatio(kSwerveModule.kSteerMotorGearReduction)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kSwerveModule.kSteerKp, kSwerveModule.kSteerKi, kSwerveModule.kSteerKd, kSwerveModule.kSteerKff, Motor.GainSlot.POSITION)
      .withOutputRange(-1,1)
      .withIZone(0.05)
      .positionWrappingEnabled(true)
      .setCalibration(m_steeringOffset - angleCalibration);

    setState(optimizedState);
  }

  public SwerveModuleState getState() {
    // Returns the velocity of the swerve drive wheel and angle
    return new SwerveModuleState(m_motorDrive.getVelocity() * (kSwerveModule.kWheelDiameterMeters * Math.PI),
        new Rotation2d(2.0 * Math.PI * m_motorSteer.getPosition()));
  }
 
  public SwerveModulePosition getPosition() {
    // Returns the position of the swerve module wheel and angle
    return new SwerveModulePosition(m_motorDrive.getPosition() * (kSwerveModule.kWheelDiameterMeters * Math.PI),
        new Rotation2d(2.0 * Math.PI * m_motorSteer.getPosition()));
  }

  public void setSpeed(double speedMetersPerSecond){
    if (!m_setupDriveDone) return;
    optimizedState.speedMetersPerSecond = speedMetersPerSecond;
    double speedRotationPerSec = speedMetersPerSecond / (kSwerveModule.kWheelDiameterMeters * Math.PI);
    m_motorDrive.setSpeed(speedRotationPerSec);
  }

  public void setState(SwerveModuleState targetState) {
    if (!m_setupDriveDone || !m_setupSteerDone) return;
    // Sets the target state of the swerve drive equal to the input state
    targetState.optimize(new Rotation2d(2.0 * Math.PI * m_motorSteer.getPosition()));
    optimizedState = targetState;
  }

  @Override
  public void periodic() {

    m_setupDriveDone = m_motorDrive.isSetupDone();
    m_setupSteerDone = m_motorSteer.isSetupDone();

    double speedcmd = m_magLimiter.calculate(optimizedState.speedMetersPerSecond);
    double anglecmd = optimizedState.angle.getRadians();

    // Motor test code over network tables
    if (testAnglePerSec != 0 || testSpeed != 0){
      testAngle = testAngle + Math.PI/180*testAnglePerSec*0.020;
      anglecmd = testAngle;
      speedcmd = testSpeed;
    }

    nt_anglecmd.set(anglecmd);
    nt_speedcmd.set(speedcmd);

    if ( m_setupDriveDone &&  m_setupSteerDone){
      // Set motor speed in rotations per second
      m_motorDrive.setSpeed(speedcmd/(Math.PI * kSwerveModule.kWheelDiameterMeters));
      // Set motor position in rotations of wheel;
      m_motorSteer.setPosition(anglecmd/(2.0*Math.PI));

      // This method will be called once per scheduler run
      nt_angle.set(m_motorSteer.getPosition()*360); // Send data in degrees over network tables
      nt_speed.set(m_motorDrive.getVelocity()*(Math.PI * Units.metersToInches(kSwerveModule.kWheelDiameterMeters))); // Send data in ft/s over network tables
      nt_i.set(m_motorDrive.getOutputCurrent());
      nt_steeri.set(m_motorSteer.getOutputCurrent());
    }
    nt_analog.set(m_analogEncoder.getValue());
  }

  public void simulationPeriodic(){
    if (!m_setupDriveDone || !m_setupSteerDone) return;
  
    if (!RobotController.isSysActive()){
      m_motorDrive.setSpeed(0);
    } else {
      //m_motorSteer.setPosition(optimizedState.angle.getRadians());
    }
  }
}
