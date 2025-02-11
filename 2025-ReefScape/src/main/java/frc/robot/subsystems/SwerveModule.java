// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.robot.Constants.kDriveTrain.kSwerveModule;
import frc.utils.FirstOrderLag;
import frc.utils.Motor;
import frc.utils.NTValues.NTDouble;
import frc.robot.Robot;
import frc.robot.Constants.CANID_t;

public class SwerveModule extends SubsystemBase {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain/wheel");
  static NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/Test");

  //public MyCANSparkMax m_motorDrive, m_motorSteer;
  //public RelativeEncoder m_encoderDrive, m_encoderSteer;
  //public SparkMaxConfig m_pidDrive = new SparkMaxConfig(); 
  //public SparkMaxConfig m_pidSteer = new SparkMaxConfig();

  //SparkClosedLoopController m_controllerDrive;
  //SparkClosedLoopController m_controllerSteer;

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
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int)kSwerveModule.kDrivingMotorCurrentLimit)
        .positionConversionFactor((Robot.isSimulation() ? 60: kSwerveModule.kDriveEncoderPositionFactor))
        .velocityConversionFactor(kSwerveModule.kDriveEncoderVelocityFactor)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kSwerveModule.kDriveKp, kSwerveModule.kDriveKi, kSwerveModule.kDriveKd, kSwerveModule.kDriveKff)
        .outputRange(-1,1)
        .iZone(0.15); 

    /* Define steer analog encoder and store calibration value */
    m_analogEncoder = new AnalogInput(CANID.encoder);
    this.angleCalibration = angleCalibration;
    m_steeringOffset = m_analogEncoder.getValue();
    nt_steeringOffset.set(m_steeringOffset);
    
    /* Define steer motor controller. */
    m_motorSteer = new Motor(CANID.steering, kSwerveModule.kSteerType, name+"_steering")
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

  public SwerveModuleState getState() {
    // Returns the velocity of the swerve drive wheel and angle
    return new SwerveModuleState(m_motorDrive.getVelocity(),
        new Rotation2d(m_motorSteer.getPosition()));
  }
 
  public void setSpeed(double speedMetersPerSecond){
    if (!m_setupDriveDone) return;
    optimizedState.speedMetersPerSecond = speedMetersPerSecond;
    m_motorDrive.setSpeed(speedMetersPerSecond);
  }

  public void setState(SwerveModuleState targetState) {
    if (!m_setupDriveDone || !m_setupSteerDone) return;
    // Sets the target state of the swerve drive equal to the input state
    targetState.optimize(new Rotation2d(m_motorSteer.getPosition()));
    optimizedState = targetState;
  }
  public SwerveModulePosition getPosition() {
    // Returns the position of the swerve module wheel and angle
    double angle;
    angle = m_motorSteer.getPosition();
    return new SwerveModulePosition(m_motorDrive.getPosition(),
        new Rotation2d(angle));
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
      m_motorDrive.setSpeed(speedcmd);
      m_motorSteer.setPosition(anglecmd);

      // This method will be called once per scheduler run
      nt_angle.set(m_motorSteer.getPosition());
      nt_speed.set(m_motorDrive.getVelocity());
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
