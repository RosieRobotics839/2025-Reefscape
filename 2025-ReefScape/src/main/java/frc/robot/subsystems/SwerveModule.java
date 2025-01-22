// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.robot.Constants.kDriveTrain.kSwerveModule;
import frc.utils.FirstOrderLag;
import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.utils.Motor;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.CANID_t;

public class SwerveModule extends SubsystemBase {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain/wheel");

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
    nt_speed, nt_i, nt_analog, nt_steeri;

  public SwerveModule(CANID_t CANID, double angleCalibration, String name) {
   
    // Setup Network Table Publishers
    nt_angleinit = table.getDoubleTopic("angle/init/"+name).publish();
    nt_angle = table.getDoubleTopic("angle/"+name).publish();
    nt_speed = table.getDoubleTopic("speed/"+name).publish();
    nt_speedcmd = table.getDoubleTopic("speedcmd/"+name).publish();
    nt_anglecmd = table.getDoubleTopic("anglecmd/"+name).publish();
    nt_i = table.getDoubleTopic("current/"+name).publish();
    nt_steeri = table.getDoubleTopic("current/"+name).publish();
    nt_analog = table.getDoubleTopic("analog/"+name).publish();

    /* Define drive motor controller. */
    /* m_motorDrive = new MyCANSparkMax(CANID.driving, MotorType.kBrushless);

    m_controllerDrive = m_motorDrive.getClosedLoopController();

    m_pidDrive
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int)kSwerveModule.kDrivingMotorCurrentLimit);
    m_pidDrive.encoder
        .positionConversionFactor((Robot.isSimulation() ? 60: kSwerveModule.kDriveEncoderPositionFactor))
        .velocityConversionFactor(kSwerveModule.kDriveEncoderVelocityFactor);
    m_pidDrive.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kSwerveModule.kDriveKp, kSwerveModule.kDriveKi, kSwerveModule.kDriveKd, kSwerveModule.kDriveKff)
        .outputRange(-1,1)
        .iZone(0.15); */
    
    /* Define steer motor controller. */
    m_motorSteer = new Motor(CANID.steering, Motor.MyMotorType.NEO, name+"_steering"); //new MyCANSparkMax(CANID.steering, MotorType.kBrushless);
    
    // m_controllerSteer  = m_motorSteer.getClosedLoopController();

    m_motorSteer
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit((int)kSwerveModule.kSteeringMotorCurrentLimit)
      .positionConversionFactor(kSwerveModule.kSteerEncoderPositionFactor)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kSwerveModule.kSteerKp, kSwerveModule.kSteerKi, kSwerveModule.kSteerKd, kSwerveModule.kSteerKff)
      .outputRange(-1,1)
      .iZone(0.05)
      .positionWrappingEnabled(true)
      .positionWrappingConfig(-Math.PI, Math.PI);
    
    /* Define steer analog encoder and store calibration value */
    m_analogEncoder = new AnalogInput(CANID.encoder);
    this.angleCalibration = angleCalibration;
    
    //m_setupDriving.ignoringDisable(true).schedule();
    m_setupSteering.ignoringDisable(true).schedule();

    setState(optimizedState);
  }

  // Most of this command sequence will be moved into a new class in Motor.java
Command m_setupDriving = Commands.sequence(
    Commands.waitUntil(() -> (m_encoderDrive = m_motorDrive.getEncoder()) != null),
    Commands.waitUntil(() -> (m_motorDrive.configure(m_pidDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)) == REVLibError.kOk),
    new InstantCommand(()-> m_setupDriveDone = true)
  );

  Command m_setupSteering = Commands.sequence(
    Commands.waitUntil(() -> {m_steeringOffset = m_analogEncoder.getValue(); nt_angleinit.set(m_steeringOffset); return true;}),
    Commands.waitUntil(() -> (m_encoderSteer = m_motorSteer.getEncoder()) != null),
    Commands.waitUntil(() -> (m_motorSteer.configure(m_pidSteer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)) == REVLibError.kOk),
    Commands.waitUntil(() -> m_encoderSteer.setPosition(-(m_steeringOffset-angleCalibration)/4096.0 * (2*Math.PI)) == REVLibError.kOk),
    new InstantCommand(()-> m_setupSteerDone = true)
  );

  public SwerveModuleState getState() {
    // Returns the velocity of the swerve drive wheel and angle
    return new SwerveModuleState(getDriveVelocity(),
        new Rotation2d(getSteerPosition()));
  }
  
  public SwerveModulePosition getPosition() {
    // Returns the position of the swerve module wheel and angle
    double angle;
    angle = getSteerPosition();
    return new SwerveModulePosition(getDrivePosition(),
        new Rotation2d(angle));
  }
  // These methods were previosuly called, replace all references with calls to Motor.java
/* 
  private double getDriveVelocity(){
    if (!m_setupDriveDone) return 0;
    return m_encoderDrive.getVelocity();
  }

  private double getDrivePosition(){
    if (!m_setupDriveDone) return 0;
    return m_encoderDrive.getPosition();
  }

  private double getSteerPosition(){
    if (!m_setupSteerDone) return 0;
    return m_encoderSteer.getPosition();
  }
*/

  public void setSpeed(double speedMetersPerSecond){
    if (!m_setupDriveDone) return;
    optimizedState.speedMetersPerSecond = speedMetersPerSecond;
    m_controllerDrive.setReference(speedMetersPerSecond,SparkMax.ControlType.kVelocity);
  }

  public void setState(SwerveModuleState targetState) {
    if (!m_setupSteerDone) return;
    //if (!m_setupDriveDone || !m_setupSteerDone) return;
    // Sets the target state of the swerve drive equal to the input state
    optimizedState = SwerveModuleState.optimize(targetState, new Rotation2d(getSteerPosition()));
    //m_pidSteer.setReference(0,CANSparkMax.ControlType.kPosition);
  }
  
  @Override
  public void periodic() {

    double speedcmd = m_magLimiter.calculate(optimizedState.speedMetersPerSecond);
    double anglecmd = optimizedState.angle.getRadians();
    
    if (/* m_setupDriveDone && */ m_setupSteerDone){
      //m_controllerDrive.setReference(speedcmd,SparkMax.ControlType.kVelocity);
      m_controllerSteer.setReference(anglecmd,SparkMax.ControlType.kPosition);

      // This method will be called once per scheduler run
      nt_angle.set(getSteerPosition());
      nt_speed.set(getDriveVelocity());
      nt_anglecmd.set(anglecmd);
      nt_speedcmd.set(speedcmd);
      //nt_i.set(m_motorDrive.getOutputCurrent());
      nt_steeri.set(m_motorSteer.getOutputCurrent());
    }
    nt_analog.set(m_analogEncoder.getValue());
  }
  
  static DCMotor m_simMotorDrive = new DCMotor(12, 2.6, 130.0, 2.70, 5676.0/60.0*2.0*Math.PI, 1);
  static DCMotor m_simMotorSteer = new DCMotor(12, 2.6, 130.0, 2.70, 5676.0/60.0*2.0*Math.PI, 1);
  static SparkMaxSim m_simDrive;
  static SparkMaxSim m_simSteer;
  public void simulationInit(){
    m_simDrive = new SparkMaxSim(m_motorDrive, m_simMotorDrive);
    m_simSteer = new SparkMaxSim(m_motorSteer, m_simMotorSteer);
  }

  public void simulationPeriodic(){
    if (!m_setupDriveDone || !m_setupSteerDone) return;
  
    if (!RobotController.isSysActive()){
      m_controllerDrive.setReference(0,SparkMax.ControlType.kVelocity);
    } else {
      m_encoderSteer.setPosition(optimizedState.angle.getRadians());
    }
    //m_simDrive.run();
    //m_simSteer.run();
  }
}
