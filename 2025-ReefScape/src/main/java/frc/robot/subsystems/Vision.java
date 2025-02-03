// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;
import frc.utils.NTValues.NTIntegerArray;

public class Vision extends SubsystemBase {

  /** Creates a new Vision instance. */
  private static Vision instance = new Vision();
  public static Vision getInstance(){
    return instance;
  }
  NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
  NetworkTable table = ntinst.getTable("roboRIO/Vision");
  NetworkTable pixytable = ntinst.getTable("PIXY");

  DoublePublisher nt_pose_x = table.getDoubleTopic("pose/x").publish();
  DoublePublisher nt_pose_y = table.getDoubleTopic("pose/y").publish();
  DoublePublisher nt_pose_t = table.getDoubleTopic("pose/t").publish();
  DoublePublisher nt_ambiguity = table.getDoubleTopic("pose/ambiguity").publish();
  DoublePublisher nt_numtargets = table.getDoubleTopic("numtargets").publish();
  DoublePublisher nt_timestamp = table.getDoubleTopic("pose/timestamp").publish();
  IntegerPublisher nt_besttarget = table.getIntegerTopic("pose/bestTarget").publish();
  public AprilTagFieldLayout aprilTagFieldLayout;

  static public PhotonCamera cam1, cam2;
  public int m_numTargets = 0;

  public static PhotonPoseEstimator photonPoseEstimatorFront, photonPoseEstimatorRear;

  // SIMULATED Vision System
  static VisionSystemSim simVision;
  static PhotonCameraSim simCamera1;
  static PhotonCameraSim simCamera2;
  static SimCameraProperties simCameraProperties;
  static Pose3d farTargetPose;
  static VisionTargetSim farTarget;

  public Vision() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.kFieldLayout);
    } catch (IOException e) {
      e.printStackTrace();
    }
    
    cam1 = new PhotonCamera(VisionConstants.frontCamera.kCameraName);
    cam2 = new PhotonCamera(VisionConstants.rearCamera.kCameraName);


    cam1.setPipelineIndex(VisionConstants.kPipelineIndex);
    cam2.setPipelineIndex(VisionConstants.kPipelineIndex);

    photonPoseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam1, VisionConstants.frontCamera.kCameraToRobot().inverse());
    photonPoseEstimatorRear = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam2, VisionConstants.rearCamera.kCameraToRobot().inverse());

    if (Robot.isSimulation()){
      simVision = new VisionSystemSim(
        VisionConstants.frontCamera.kCameraName
      );

      simCameraProperties = new SimCameraProperties();
      simCameraProperties.setCalibration(
        VisionConstants.frontCamera.kSimCamResolutionW,
        VisionConstants.frontCamera.kSimCamResolutionH,
        new Rotation2d(VisionConstants.frontCamera.kCamDiagFOV)
      );

      simCameraProperties.setCalibError(0.0, 0.0);
      simCameraProperties.setFPS(45);
      simCameraProperties.setAvgLatencyMs(30);
      simCameraProperties.setLatencyStdDevMs(4);
      
      simCamera1 = new PhotonCameraSim(cam1, simCameraProperties, VisionConstants.kMinTargetArea, VisionConstants.frontCamera.kSimMaxLEDRange);
      simCamera2 = new PhotonCameraSim(cam2, simCameraProperties, VisionConstants.kMinTargetArea, VisionConstants.frontCamera.kSimMaxLEDRange);
      simCamera1.enableDrawWireframe(true);
      simCamera2.enableDrawWireframe(true);
      simVision.addCamera(simCamera1, VisionConstants.frontCamera.kCameraToRobot().inverse()); 
      simVision.addCamera(simCamera2, VisionConstants.rearCamera.kCameraToRobot().inverse());
      simVision.addAprilTags(aprilTagFieldLayout);
    }
  }

  @Override
  public void periodic() {
    if (true) return;
    // This method will be called once per scheduler run
    @SuppressWarnings("unused")
    var res = cam1.getLatestResult();

    if (res.hasTargets()) {
      
      photonPoseEstimatorFront.setReferencePose(PoseEstimator.getInstance().m_finalPose3d);
      Optional<EstimatedRobotPose> robotPose = photonPoseEstimatorFront.update();
      
      nt_ambiguity.set(res.getBestTarget().getPoseAmbiguity());
      if (robotPose.isPresent()){
        Pose3d camPose = robotPose.get().estimatedPose;
        nt_timestamp.set(robotPose.get().timestampSeconds);
        nt_pose_x.set(camPose.getX());
        nt_pose_y.set(camPose.getY());
        nt_pose_t.set(camPose.getRotation().toRotation2d().getRadians());
        if (res.getBestTarget().getPoseAmbiguity() < VisionConstants.kMaxAmbiguity){
          double latency_ms = Math.max(0,Math.min(VisionConstants.kMaxLatencyCompensationMillis, res.getLatencyMillis()));
          PoseEstimator.getInstance().addVisionMeasurement(robotPose, VisionConstants.kExtraLatencyMillis + latency_ms);
        }
      }

      nt_besttarget.set(res.getBestTarget().getFiducialId());
    }
    m_numTargets = res.targets.size();

    res = cam2.getLatestResult();

    if (res.hasTargets()) {
      
      photonPoseEstimatorRear.setReferencePose(PoseEstimator.getInstance().m_finalPose3d);
      Optional<EstimatedRobotPose> robotPose = photonPoseEstimatorRear.update();
      
      nt_ambiguity.set(res.getBestTarget().getPoseAmbiguity());
      if (robotPose.isPresent()){
        Pose3d camPose = robotPose.get().estimatedPose;
        nt_timestamp.set(robotPose.get().timestampSeconds);
        nt_pose_x.set(camPose.getX());
        nt_pose_y.set(camPose.getY());
        nt_pose_t.set(camPose.getRotation().toRotation2d().getRadians());
        if (res.getBestTarget().getPoseAmbiguity() < VisionConstants.kMaxAmbiguity){
          PoseEstimator.getInstance().addVisionMeasurement(robotPose, VisionConstants.kExtraLatencyMillis + res.getLatencyMillis());
        }
      }

      nt_besttarget.set(res.getBestTarget().getFiducialId());
    }
    m_numTargets += res.targets.size();
    nt_numtargets.set(m_numTargets);

    // Run the periodic function for the Pixy Camera
    pixyPeriodic();
  }

  @Override
  public void simulationPeriodic() {
    simVision.update(PoseEstimator.getInstance().m_sim_actualPose);
  }
  // PIXY Camera - Get Data with lambda when new data arrives :)
  double m_pixyNBlock = NTDouble.create(0, pixytable,"nblocks",val->m_pixyNBlock=val);
  double m_heartbeat = NTDouble.create(0, pixytable,"heartbeat",val->m_heartbeat=val);
  double m_frameWidth = NTDouble.create(316, pixytable,"frameWidth",val->m_frameWidth=val);
  double m_frameHeight = NTDouble.create(208, pixytable,"frameHeight",val->m_frameHeight=val);
  double m_frame = NTDouble.create(0, pixytable,"frame",val->m_frame=val);
  long [] m_blockData = NTIntegerArray.create(new long []{}, pixytable,"blockdata",val->m_blockData=val);

  boolean m_targetIsLocked = NTBoolean.create(false, pixytable, "targetIsLocked",val->m_targetIsLocked=val);
  double m_lockedBlockWidth = NTDouble.create(0, pixytable, "lockedBlockWidth", val->m_lockedBlockWidth=val);
  double m_lockedBlockX = NTDouble.create(0, pixytable, "lockedBlockX", val->m_lockedBlockX=val);
  double m_lockedBlockY = NTDouble.create(0, pixytable, "lockedBlockY", val->m_lockedBlockY=val);
  double m_lockedBlockAge = NTDouble.create(0, pixytable, "lockedBlockAge", val->m_lockedBlockAge=val);
  double m_lockedBlockIndex = NTDouble.create(0, pixytable, "lockedBlockIndex", val->m_lockedBlockIndex=val);
  double m_frameCenterOffset = NTDouble.create(0, pixytable, "frameCenterOffset", val->m_frameCenterOffset=val);
 
  class PixyBlock {
    public int signature;
    public int x;
    public int y;
    public int width;
    public int height;
    public int angle;
    public int index; // Pixy assigns and tracks objects by an index
    public int age; // Age that the pixy has been tracking the object, up to value of 255;
  }  

  double m_heartbeatPrev = 0;
  List<PixyBlock> m_pixyblocks = new ArrayList<PixyBlock>();
  boolean targetIsLocked = false;
  int lockedTargetIndex;
  int idealTargetIndex;
  double targetOffset;
  double m_pixyTargetAngle;
  double m_heartbeattimeout = 0;
  double frameCenterOffset = 157;

  private void pixyPeriodic(){
    idealTargetIndex = -1;

    m_heartbeattimeout = Math.max(0,m_heartbeattimeout-0.02); // Subtract 20ms from the watchdog

    // Interpret data from NetworkTables
    if (m_heartbeat != m_heartbeatPrev){
      
      // Reset timeout since new data arrived, feed the watchdog.
      m_heartbeattimeout = VisionConstants.kPixyTimeout;

      // Reset pixy block list for new data
      m_pixyblocks = new ArrayList<PixyBlock>();
      
      int blocksize = 8;
      int maxBlockWidth = 0;
      // Check that data size is correct for the number of blocks
      if (m_blockData.length > 0 && m_blockData.length == blocksize*m_pixyNBlock){
        for (int i=0; i<m_pixyNBlock; i++){
          int j=0;
          PixyBlock block = new PixyBlock();
          block.signature = (int)m_blockData[blocksize*i+j++];
          block.x = (int)m_blockData[blocksize*i+j++];
          block.y = (int)m_blockData[blocksize*i+j++];
          block.width = (int)m_blockData[blocksize*i+j++];
          block.height = (int)m_blockData[blocksize*i+j++];
          block.angle = (int)m_blockData[blocksize*i+j++];
          block.index = (int)m_blockData[blocksize*i+j++];
          block.age = (int)m_blockData[blocksize*i+j++];

          // Keep track of widest block
          if (block.width > maxBlockWidth){
            idealTargetIndex = block.index;
            maxBlockWidth = block.width;
          }

          m_pixyblocks.add(block);
        }
      }
      m_heartbeatPrev = m_heartbeat;
    }

    // reset pixy block list after timeout
    if (m_heartbeattimeout <= 0){
      m_pixyblocks = new ArrayList<PixyBlock>();
    }
  }
  public void findPixyTarget(){

    // Make sure locked block is still there, if not then unlock it.
    PixyBlock lockedBlock = m_pixyblocks.stream().filter(block -> lockedTargetIndex==block.index).findAny().orElse(null);
    if (targetIsLocked && lockedBlock == null){
      unlockTarget();
    }

    // Find ideal block and lock the index 
    PixyBlock idealBlock = m_pixyblocks.stream().filter(block -> idealTargetIndex==block.index).findAny().orElse(null);
    if (!targetIsLocked && idealBlock != null){
      targetIsLocked = true; // Lock the target in
      lockedTargetIndex = idealTargetIndex;
      lockedBlock = idealBlock;
      m_targetIsLocked = targetIsLocked;
      m_lockedBlockAge = lockedBlock.age;
      m_lockedBlockX = lockedBlock.x;
      m_lockedBlockY = lockedBlock.y;
      m_lockedBlockIndex = lockedBlock.index;
      m_lockedBlockWidth = lockedBlock.width;
    }
    


    // Calc targeting information if locked and valid
    if (targetIsLocked && lockedBlock != null){
      targetOffset = ((frameCenterOffset - lockedBlock.x) + (lockedBlock.width / 2)); // frameCenterOffset default is 157, half the frame width
      m_pixyTargetAngle = lockedBlock.angle;
    } else {
      targetOffset = 0;
      m_pixyTargetAngle = 0;
    }

  }

  public void unlockTarget(){
    targetIsLocked = false;
  }
}
