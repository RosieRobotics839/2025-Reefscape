// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.kDriveTrain;
import frc.utils.VectorUtils;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;

public class PoseEstimator extends SubsystemBase {

  private static final PoseEstimator estimator = new PoseEstimator();
  public static PoseEstimator getInstance(){
    return estimator;
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/PoseEstimator");

  // Setup Network Table Publishers
  DoublePublisher nt_final_x = table.getDoubleTopic("final_x").publish();
  DoublePublisher nt_final_y = table.getDoubleTopic("final_y").publish();
  DoublePublisher nt_final_t = table.getDoubleTopic("final_t").publish();
  DoublePublisher nt_pred_x = table.getDoubleTopic("pred_x").publish();
  DoublePublisher nt_pred_y = table.getDoubleTopic("pred_y").publish();
  DoublePublisher nt_pred_t = table.getDoubleTopic("pred_t").publish();
  DoublePublisher nt_gyro_res = table.getDoubleTopic("gyro_res").publish();
  DoublePublisher nt_vision_resx = table.getDoubleTopic("vis_res_x").publish();
  DoublePublisher nt_vision_resy = table.getDoubleTopic("vis_res_y").publish();
  DoublePublisher nt_vision_rest = table.getDoubleTopic("vis_res_t").publish();

  DoubleArrayPublisher nt_simPose_t = table.getDoubleArrayTopic("simPose").publish();

  private Gyro m_gyro = Gyro.getInstance();
  private DriveTrain m_drivetrain = DriveTrain.getInstance();
  
  public boolean m_visionIsValid = false;
  public Pose2d m_sim_actualPose = new Pose2d(0,0,new Rotation2d(0));
  public Pose2d m_finalPose = new Pose2d();
  private Pose2d m_predictedPose = new Pose2d();
  private Pose2d m_tempPose = new Pose2d();
  public Pose3d m_finalPose3d = new Pose3d();
  public double m_lastTime;

  public Rotation2d m_visionTheta = new Rotation2d();
  private Pose3d m_visionPose3d;
  private Pose2d m_visionPose2d = new Pose2d();
  private double m_visionTimestamp;
  private double m_visionLastTimestamp;

  Pose2d m_visionPoseResidual = new Pose2d();
  Translation2d m_visionPoseCorrection_pos = new Translation2d();
  double m_visionPoseCorrection_rot = 0;
  public double m_gyroResidual = 0;
  SwerveModulePosition [] m_previousModulePositions;

  Twist2d m_predictedTwist;

  public final Field2d m_field = new Field2d();
  FieldObject2d m_fieldsimpose = m_field.getObject("SimPose");

  /** Creates a new PoseEstimator. */
  public PoseEstimator() {
    m_previousModulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(0,new Rotation2d(0)),
      new SwerveModulePosition(0,new Rotation2d(0)),
      new SwerveModulePosition(0,new Rotation2d(0)),
      new SwerveModulePosition(0,new Rotation2d(0))
    };

    // Display pose estimate on the Field2d widget in Glass 
    SmartDashboard.putData("Field", m_field);
  }

  public void addVisionMeasurement(Optional<EstimatedRobotPose> observedPose, double latency_ms){
    if (observedPose.isPresent()){
      m_visionPose3d = observedPose.get().estimatedPose;
      m_visionTimestamp = observedPose.get().timestampSeconds;

      m_visionPose2d = new Pose2d(m_visionPose3d.getX(), m_visionPose3d.getY(), m_visionPose3d.getRotation().toRotation2d());
      double timescale = latency_ms/20.0;
      Twist2d m_latencyCompensation = new Twist2d(m_predictedTwist.dx*timescale, m_predictedTwist.dy*timescale, m_predictedTwist.dtheta*timescale);
      m_visionPose2d = m_visionPose2d.exp(m_latencyCompensation);
    }
  }

  public void reset(){
    if (FlightStick.m_blueAlly){
      m_finalPose = new Pose2d(1,2.5,new Rotation2d(0));
      m_sim_actualPose = new Pose2d(1,2.5,new Rotation2d(0));
    } else {
      m_finalPose = new Pose2d(15.5,2.5,new Rotation2d(Math.PI));
      m_sim_actualPose = new Pose2d(15.5,2.5,new Rotation2d(Math.PI));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_drivetrain = DriveTrain.getInstance();

    SmartDashboard.putNumber("Gyro", m_finalPose.getRotation().getDegrees());
    // More accurate method using actual swerve drive positions and forward kinematics calculation

    // Reset Pose Estimation to (0,0,0) on RoboRio USER button press
    if (!RobotController.isSysActive() && (RobotController.getUserButton())){        
      reset();
    }

    // Allow override of field Pose estimation via glass field object
    Pose2d fieldPose = m_field.getRobotPose();
    if (
      Math.abs(fieldPose.getX() - m_finalPose.getX()) > .001 || 
      Math.abs(fieldPose.getY() - m_finalPose.getY()) > .001 || 
      Math.abs(fieldPose.getRotation().getRadians() - m_finalPose.getRotation().getRadians()) > 0.001
    ){
      m_finalPose = fieldPose;
      if (Robot.isSimulation()){
        m_sim_actualPose = fieldPose;
      }
    }
    
    // Update estimator based on SwerveModule actual states
    SwerveModulePosition[] modulePositions;
    if (m_drivetrain.m_motorSetupDone){
      modulePositions = new SwerveModulePosition[]{
      m_drivetrain.frontLeft.getPosition(),
      m_drivetrain.frontRight.getPosition(), 
      m_drivetrain.rearLeft.getPosition(),
      m_drivetrain.rearRight.getPosition()};
    } else {
      modulePositions = m_previousModulePositions;
    }

    var moduleDeltas = new SwerveModulePosition[modulePositions.length];
    for (int index = 0; index < modulePositions.length; index++) {
      var current = modulePositions[index];
      var previous = m_previousModulePositions[index];

      moduleDeltas[index] =
          new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
      previous.distanceMeters = current.distanceMeters;
    }

    m_predictedTwist = kDriveTrain.kDriveKinematics.toTwist2d(moduleDeltas);
    m_predictedTwist = new Twist2d(m_predictedTwist.dx*PoseConstants.kDriveSlip, m_predictedTwist.dy*PoseConstants.kDriveSlip, m_predictedTwist.dtheta*PoseConstants.kDriveSlip);
    m_predictedPose = m_finalPose.exp(m_predictedTwist);

    if (Robot.isSimulation()){
      m_sim_actualPose = m_sim_actualPose.exp(m_predictedTwist);
      nt_simPose_t.set(new double[]{m_sim_actualPose.getX(),m_sim_actualPose.getY(),m_sim_actualPose.getRotation().getRadians()});
    }
    
    // Temporary pose which will be modified by measured corrections, but initially based on the odometry model
    m_tempPose = m_predictedPose;

    // Gyro Correction Applied only for Rotation correction to Predicted Odometry Pose
    double gyroAngle = m_gyro.getYaw();
    m_gyroResidual = VectorUtils.angleDifference(gyroAngle,m_tempPose.getRotation().getRadians());
    double gyroCorrection = 0;
    if (m_gyro.getStatus()){
      gyroCorrection = m_gyroResidual*(PoseConstants.kGyroWeight);
    }
    m_tempPose = new Pose2d(m_tempPose.getX(),m_tempPose.getY(), new Rotation2d(m_tempPose.getRotation().getRadians()+gyroCorrection));

    // Vision System correction applied Primarily for X,Y coordinate correction to estimated pose.
    if (m_visionTimestamp != m_visionLastTimestamp){
      m_visionIsValid = true;
      m_visionTheta = m_visionPose2d.getRotation();
      m_visionPoseResidual = VectorUtils.poseDiff(m_visionPose2d,m_tempPose);
      m_visionPoseCorrection_pos = m_visionPoseResidual.times(PoseConstants.kVisionWeightPos).getTranslation();
      m_visionPoseCorrection_rot = (m_visionPoseResidual.getRotation().getRadians())*PoseConstants.kVisionWeightRot;
    } else {
      m_visionIsValid = false;
      m_visionTheta = null;
      m_visionPoseCorrection_pos = new Translation2d(0,0);
      m_visionPoseCorrection_rot = m_visionPoseCorrection_rot*PoseConstants.kVisionWeightRotDecay;
    }
    if (!Double.isNaN(m_tempPose.getX()) && !Double.isNaN(m_tempPose.getY()) && !Double.isNaN(m_tempPose.getRotation().getRadians())){
      m_tempPose = new Pose2d(m_tempPose.getX()+m_visionPoseCorrection_pos.getX(),
                          m_tempPose.getY()+m_visionPoseCorrection_pos.getY(),
                          new Rotation2d(m_tempPose.getRotation().getRadians()+m_visionPoseCorrection_rot));
    }
    m_visionLastTimestamp = m_visionTimestamp;
   
    if (!Double.isNaN(m_tempPose.getX()) && !Double.isNaN(m_tempPose.getY()) && !Double.isNaN(m_tempPose.getRotation().getRadians())){
      m_finalPose = m_tempPose;
    }
    m_field.setRobotPose(m_finalPose);
    
    if (Robot.isSimulation()){
      m_fieldsimpose.setPose(m_sim_actualPose);
    }

    nt_final_x.set(m_finalPose.getX());
    nt_final_y.set(m_finalPose.getY());
    nt_final_t.set(m_finalPose.getRotation().getRadians());
    nt_gyro_res.set(m_gyroResidual);
    nt_pred_x.set(m_predictedPose.getX());
    nt_pred_y.set(m_predictedPose.getY());
    nt_pred_t.set(m_predictedPose.getRotation().getRadians());
    nt_vision_resx.set(m_visionPoseResidual.getX());
    nt_vision_resy.set(m_visionPoseResidual.getY());
    nt_vision_rest.set(m_visionPoseResidual.getRotation().getRadians());

  }
}
