// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.utils.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.kChassis;
import frc.robot.Constants.kDriveTrain;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTDoubleArray;

public class DriveTrain extends SubsystemBase {

  private static DriveTrain instance = new DriveTrain();

  public static DriveTrain getInstance(){
    return instance;
  }

  private Controller driveController = Controller.getDriveInstance();
  private FlightStick driveFlightStick = FlightStick.getDriveInstance();

  NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/CAUTION/TestInput/Drivetrain");
  NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain");

  public double lastTime = System.currentTimeMillis();

  public double m_autoSpeed = 0;
  public SlewRateLimiter m_autoAccelLimiter = new SlewRateLimiter(DriveConstants.kAutoAccelLimiter, -1E9, 0);

  private double m_forward, m_left, m_rotate;
  private double m_forwardcmd, m_leftcmd, m_rotatecmd;
  private boolean m_controllerInputActive = false;

  private double m_maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond[DriveConstants.kMaxSpeedDefault];
  private double m_maxRotate = DriveConstants.kMaxRotationVelocity[DriveConstants.kMaxSpeedDefault];
  
  public void setMaxSpeed(double maxSpeedMetersPerSecond){
    m_maxSpeed = maxSpeedMetersPerSecond;
  }
  public void setMaxRotate(double maxRotateRadiansPerSecond){
    m_maxRotate = maxRotateRadiansPerSecond;
  }

  // Create new swerve modules
  public SwerveModule frontLeft = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_FrontLeft, kDriveTrain.kSwerveModule.kCalibrationFrontLeft, "frontLeft");
  public SwerveModule frontRight = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_FrontRight, kDriveTrain.kSwerveModule.kCalibrationFrontRight, "frontRight");
  public SwerveModule rearLeft = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_RearLeft, kDriveTrain.kSwerveModule.kCalibrationRearLeft, "rearLeft");
  public SwerveModule rearRight = new SwerveModule(kDriveTrain.kSwerveModule.kCANID_RearRight, kDriveTrain.kSwerveModule.kCalibrationRearRight, "rearRight");

  public SwerveModule [] modules = {frontLeft, frontRight, rearLeft, rearRight};

  SwerveModuleState swerveModuleStates[] = kDriveTrain.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));

  /* ----------------------------------------------------------------------------------------------------- */

  double headingRotateOffset = 0;
  public double m_targetHeading = 0;
  public double m_currentHeading;
 
  public double headingError;

  final DoublePublisher nt_forward = table.getDoubleTopic("forward").publish();
  final DoublePublisher nt_left = table.getDoubleTopic("left").publish();
  final DoublePublisher nt_rotate = table.getDoubleTopic("rotate").publish();
  final DoublePublisher nt_speed = table.getDoubleTopic("speed").publish();
  final DoublePublisher nt_distance = table.getDoubleTopic("distance").publish();
  final BooleanPublisher nt_fieldCentricDriving = table.getBooleanTopic("HeadingPID/fieldCentricDriving").publish();

  public PIDController m_headingPID;

  static NTDoubleArray nt_poseTarget;

  /** Creates a new DriveTrain. */
  private DriveTrain() {
    m_headingPID = new PIDController(DriveConstants.kPIDHeadingKp, DriveConstants.kPIDHeadingKi, DriveConstants.kPIDHeadingKd);
    m_headingPID.publish(table,"HeadingPID");
    m_headingPID.setNTScale(Units.radiansToDegrees(1));
    m_headingPID.setIntegratorRange(-DriveConstants.kPIDHeadingIntegratorRange, DriveConstants.kPIDHeadingIntegratorRange);
    m_headingPID.enableContinuousInput(0, Units.degreesToRadians(360));
    
    NTDoubleArray.create(new double[]{0,0,0},testtable,"driveToPose",
      val->m_poseQueue.offer(
        new Pose2d(
          Units.feetToMeters(val[0]),
          Units.feetToMeters(val[1]),
          new Rotation2d(Units.degreesToRadians(val[2]))
        )
      )
    );
  }

  public LinkedList<Pose2d> m_poseQueue = new LinkedList<Pose2d>();

  public boolean isStopped(){
    return 
    Math.abs(frontLeft.m_encoderDrive.getVelocity()) <= DriveConstants.kStoppedRatio*DriveConstants.kAutoMaxSpeed &&
    Math.abs(frontRight.m_encoderDrive.getVelocity()) <= DriveConstants.kStoppedRatio*kDriveTrain.DriveConstants.kAutoMaxSpeed &&
    Math.abs(rearLeft.m_encoderDrive.getVelocity()) <=  DriveConstants.kStoppedRatio*kDriveTrain.DriveConstants.kAutoMaxSpeed &&
    Math.abs(rearRight.m_encoderDrive.getVelocity()) <= DriveConstants.kStoppedRatio*kDriveTrain.DriveConstants.kAutoMaxSpeed;
  }
  public void Drive(Twist2d movement){
    m_forwardcmd = movement.dx;
    m_leftcmd = movement.dy;
    m_rotatecmd = movement.dtheta;
  } 

  public void Drive(double _forward, double _left, double _rotate){
    m_forwardcmd = _forward;
    m_leftcmd = _left;
    m_rotatecmd = _rotate;
  }

  public void PublishPoseQueue(){
    List<Pose2d> poseQueue = m_poseQueue.stream().map((a)->new Pose2d(a.getTranslation(),
      (a.getRotation()==null ? new Rotation2d(0) : a.getRotation()))).collect(Collectors.toList());
    poseQueue.add(0, PoseEstimator.getInstance().m_finalPose);
    PoseEstimator.getInstance().m_field.getObject("AutoTraj").setPoses(poseQueue);
  }

  private void RunDrive() {
    // Follow Drive to Pose Queue, unless controller input is active, which clears the queue in periodic().
    if (!m_poseQueue.isEmpty()){ 
      Twist2d movement;
      if (VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose, m_poseQueue.peek(), (m_poseQueue.size() > 1 ? DriveConstants.kMidPointAccuracyFactor : 1) * DriveConstants.kAutoToleranceDistance, (m_poseQueue.size() > 1 ? DriveConstants.kMidPointAccuracyFactor : 1) * DriveConstants.kAutoToleranceAngle)){
        // Reached Target Pose
        PublishPoseQueue();
        m_poseQueue.poll();
        movement = new Twist2d(0,0,0);
        Drive(movement);
        nt_distance.set(0);
      } else {
        // Moving to Target Pose
        Pose2d diff = VectorUtils.poseDiff(m_poseQueue.peek(),PoseEstimator.getInstance().m_finalPose);
        double distance = Math.max(0,VectorUtils.poseDiff(m_poseQueue.peekLast(),PoseEstimator.getInstance().m_finalPose).getTranslation().getNorm() - kChassis.kWheelBase/2.0);
        nt_distance.set(distance);
        m_autoSpeed = m_autoAccelLimiter.calculate(Math.max(Math.min(1,distance/(DriveConstants.kAutoSlowDist))*DriveConstants.kAutoMaxSpeed, DriveConstants.kAutoMinSpeed));
        Translation2d vector = VectorUtils.vectorInDirectionOf(diff, m_autoSpeed);
        movement = new Twist2d(vector.getX(), vector.getY(), 0);
        Drive(movement);
        if (m_poseQueue.peek().getRotation() != null && distance < DriveConstants.kAutoTurnToPoseDistance){
          double angle = (m_poseQueue.peek().getRotation().getRadians());
          setTargetHeading(angle);
        }
      }
    } else if(!DriverStation.isTeleopEnabled()){
      // If no pose to go to navigate to in autonomous, stop!
      Drive(new Twist2d(0,0,0));
      nt_distance.set(0);
      m_autoSpeed = m_autoAccelLimiter.calculate(0);
    } else {
      m_autoSpeed = m_autoAccelLimiter.calculate(0);
    }

    double dt = (System.currentTimeMillis()-lastTime)*1e-3;
    dt = (dt < 0.2 ? dt : 0.02);
    lastTime = System.currentTimeMillis();

    /* Rotation Control Logic */
    m_targetHeading += m_rotatecmd * dt * m_maxRotate;
    m_targetHeading = MathUtil.inputModulus(m_targetHeading, 0, 2*Math.PI);
    m_currentHeading = PoseEstimator.getInstance().m_finalPose.getRotation().getRadians();
    m_currentHeading = MathUtil.inputModulus(m_currentHeading, 0, 2*Math.PI);
    double directRotate = (Autonomous.getInstance().m_aimPoint == null ? DriveConstants.kRotationDirectControlRatio : 0);
    m_rotate = DriveConstants.kPIDHeadingMaxRotSpeed*Math.min(1.0,Math.max(-1.0,m_headingPID.calculate(m_currentHeading, m_targetHeading))) + m_maxRotate * directRotate * m_rotatecmd;
    
    // Translation Control Logic */
    if (!m_poseQueue.isEmpty()){ // Driving autonomously uses calculated speeds
      m_forward = (m_forwardcmd * Math.cos(-m_currentHeading) + m_leftcmd * -Math.sin(-m_currentHeading)); 
      m_left    = (m_forwardcmd * Math.sin(-m_currentHeading) + m_leftcmd *  Math.cos(-m_currentHeading));
    } else if (OperatorConstants.kFieldCentricDriving){ 
      m_forward = m_maxSpeed * (m_forwardcmd * Math.cos(-m_currentHeading) + m_leftcmd * -Math.sin(-m_currentHeading)); 
      m_left    = m_maxSpeed * (m_forwardcmd * Math.sin(-m_currentHeading) + m_leftcmd *  Math.cos(-m_currentHeading));
    } else {
      m_forward = m_maxSpeed * m_forwardcmd;
      m_left    = m_maxSpeed * m_leftcmd;
    }
    Translation2d translationReq = new Translation2d(m_forward, m_left);

    nt_speed.set(translationReq.getNorm());

    // Calculate new Swerve Module states using Reverse Kinematics
    swerveModuleStates = kDriveTrain.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(translationReq.getX(), translationReq.getY(), m_rotate));
    
    // Keep direction control and slow down if exceeding maximum wheel speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kAttainableMaxSpeed);

    double speed = VectorUtils.SRSS(swerveModuleStates[0].speedMetersPerSecond, swerveModuleStates[1].speedMetersPerSecond, swerveModuleStates[2].speedMetersPerSecond, swerveModuleStates[3].speedMetersPerSecond);
    
    if (speed < DriveConstants.kMinDriveSpeed){
      forEachSwerveModule(m->m.setSpeed(0));
    } else {
      // Define Swerve Module States, make sure order matches Kinematics Definition in Constants.java file
      frontLeft.setState(swerveModuleStates[0]);
      frontRight.setState(swerveModuleStates[1]);
      rearLeft.setState(swerveModuleStates[2]);
      rearRight.setState(swerveModuleStates[3]);
    }
    
    /* Update drive commands on network tables */
    nt_forward.set(m_forward);
    nt_left.set(m_left);
    nt_rotate.set(m_rotate);
  }
  
  public boolean atTargetHeading(double tolerance) {
    return Math.abs(VectorUtils.angleDifference(m_targetHeading,m_currentHeading)) < tolerance;
  }

  public boolean atTargetHeading() {
    return atTargetHeading(DriveConstants.kAutoToleranceAngle);
  }

  public void setTargetHeading(double targetHeading) {
    this.m_targetHeading = targetHeading;
  }

  public void syncHeading() {
    this.m_targetHeading = m_currentHeading;
  }

  public double getTargetHeading(){
    return m_targetHeading;
  }

  public void simulationInit(){
    frontLeft.simulationInit();
    frontRight.simulationInit();
    rearLeft.simulationInit();
    rearRight.simulationInit();
  }
  
  Boolean m_motorSetupDone = false;
  boolean testBool = true;
  @Override
  public void periodic() {
    if (!m_motorSetupDone){
      testBool = true;
      forEachSwerveModule((s)->testBool = testBool && s.m_setupDriveDone && s.m_setupSteerDone);
      m_motorSetupDone = testBool;
    }

    // Reset target heading on USER button press
    if (!RobotController.isSysActive()){
      m_targetHeading = PoseEstimator.getInstance().m_finalPose.getRotation().getRadians();
    }

    // Calculate Drive Inputs from Controller
    if (DriverStation.isTeleopEnabled()){
      if (OperatorConstants.kDriverControllerIsFlightStick){
        driveFlightStick.Translate();
        Drive(FlightStick.forward, FlightStick.left, FlightStick.rotate);
        m_controllerInputActive = VectorUtils.SRSS(FlightStick.forward, FlightStick.left, FlightStick.rotate) > OperatorConstants.kControllerActiveThreshold;
      } else {
        driveController.Translate();
        Drive(Controller.forward, Controller.left, Controller.rotate);
        m_controllerInputActive = VectorUtils.SRSS(Controller.forward, Controller.left, Controller.rotate) > OperatorConstants.kControllerActiveThreshold;
      }
    }

    // If controller input is detected clear the pose driving queue
    if (m_controllerInputActive){
      m_poseQueue.clear();
      PublishPoseQueue();
    }

    // Reset drive inputs if Robot becomes disabled. Must be last change before RunDrive();
    if (DriverStation.isDisabled()){
      Drive(0,0,0);
      m_targetHeading = m_currentHeading;
    }

    RunDrive();
  }

  public static void forEachSwerveModule(Consumer<SwerveModule> lambda){
    var dt = DriveTrain.getInstance();
    SwerveModule [] motors = {dt.frontLeft, dt.frontRight, dt.rearLeft, dt.rearRight};
    for (SwerveModule m : motors){
      lambda.accept(m);
    }
  }
}