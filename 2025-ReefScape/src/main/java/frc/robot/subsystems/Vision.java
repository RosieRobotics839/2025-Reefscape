// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

  /** Creates a new Vision instance. */
  private static Vision instance = new Vision();
  public static Vision getInstance(){
    return instance;
  }
  NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
  NetworkTable table = ntinst.getTable("roboRIO/Vision");
  NetworkTable pixytable = ntinst.getTable("PIXY");
  
  NTCamResult nt_posefront = new NTCamResult(table, "poseFront");
  NTCamResult nt_poserear = new NTCamResult(table, "poseRear");

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

  public void PublishAprilTags(){
    List<Pose2d> tags = aprilTagFieldLayout.getTags().stream().map(
          (a)->new Pose2d(a.pose.getTranslation().toTranslation2d(),
                          a.pose.getRotation().toRotation2d()
                          )).collect(Collectors.toList()
    );
    PoseEstimator.getInstance().m_field.getObject("AprilTags").setPoses(tags);
  }

  public Vision() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.kFieldLayout);
      PublishAprilTags();
    } catch (IOException e) {
      e.printStackTrace();
    }
    
    cam1 = new PhotonCamera(VisionConstants.frontCamera.kCameraName);
    cam2 = new PhotonCamera(VisionConstants.rearCamera.kCameraName);

    cam1.setPipelineIndex(VisionConstants.kPipelineIndex);
    cam2.setPipelineIndex(VisionConstants.kPipelineIndex);

    photonPoseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, (Robot.isSimulation() ? PoseStrategy.MULTI_TAG_PNP_ON_RIO : PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR), VisionConstants.frontCamera.kCameraToRobot().inverse());
    photonPoseEstimatorRear = new PhotonPoseEstimator(aprilTagFieldLayout, (Robot.isSimulation() ? PoseStrategy.MULTI_TAG_PNP_ON_RIO : PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR), VisionConstants.rearCamera.kCameraToRobot().inverse());

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

  /**
   * Executes code to process the last camera result from Photonvision that contains targets from a PhotonCamera and returns the CamResult
   * 
   * @param camera PhotonCamera to process
   * @param poseEst PhotonPoseEstimator to update
   */
  private CamResult processCamera(PhotonCamera camera, final PhotonPoseEstimator poseEst){
    int numTargets = 0;
    Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
    Optional<EstimatedRobotPose> robotPose = Optional.empty();

    var results = camera.getAllUnreadResults();
    ListIterator<PhotonPipelineResult> res_iter = results.listIterator(results.size());

    while (res_iter.hasPrevious()){
      // get latest camera result
      PhotonPipelineResult result = res_iter.previous(); 
     
      if (result.hasTargets()) {
        numTargets = result.targets.size();
        bestTarget = Optional.of(result.getBestTarget());

        robotPose = poseEst.update(
          result,
          camera.getCameraMatrix(),
          camera.getDistCoeffs()
        );

        double ambiguity = result.getBestTarget().getPoseAmbiguity();
        if ((ambiguity > 0 || (DriverStation.isDisabled() && ambiguity == 0)) && ambiguity < VisionConstants.kMaxAmbiguity){

          PoseEstimator.getInstance().addVisionMeasurement(robotPose,result.metadata.getCaptureTimestampMicros());
        }
      }
    }
    return new CamResult(numTargets,bestTarget,robotPose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    photonPoseEstimatorFront.setReferencePose(PoseEstimator.getInstance().m_finalPose3d);
    photonPoseEstimatorRear.setReferencePose(PoseEstimator.getInstance().m_finalPose3d);

    var cam1result = processCamera(cam1, photonPoseEstimatorFront);
    var cam2result = processCamera(cam2, photonPoseEstimatorRear);
    
    nt_posefront.set(cam1result);
    nt_poserear.set(cam2result);
    
    // Run the periodic function for the Pixy Camera
    // pixyPeriodic();
  }

  @Override
  public void simulationPeriodic() {
    simVision.update(PoseEstimator.getInstance().m_sim_actualPose);
  }

  /**
   * A class to hold information from a processed camera target
   */
  class CamResult {
    int numTargets;
    Optional<PhotonTrackedTarget> bestTarget;
    Optional<EstimatedRobotPose> pose;
    public CamResult(int numTargets, Optional<PhotonTrackedTarget> bestTarget, Optional<EstimatedRobotPose> pose){this.numTargets=numTargets; this.bestTarget=bestTarget; this.pose = pose;}
  };

  class NTCamResult {
    DoublePublisher nt_pose_x, nt_pose_y, nt_pose_t, nt_ambiguity, nt_timestamp;
    IntegerPublisher nt_besttarget, nt_targets;
    NTCamResult (NetworkTable _table, String _name){
      nt_pose_x = _table.getDoubleTopic(_name+"/x").publish();
      nt_pose_y = _table.getDoubleTopic(_name+"/y").publish();
      nt_pose_t = _table.getDoubleTopic(_name+"/t").publish();
      nt_ambiguity = _table.getDoubleTopic(_name+"/ambiguity").publish();
      nt_besttarget = _table.getIntegerTopic(_name+"/bestTarget").publish();
      nt_targets = _table.getIntegerTopic(_name+"/targets").publish();
      nt_timestamp = _table.getDoubleTopic(_name+"/timestamp").publish();
    }
    public void set(CamResult result){
      nt_targets.set(result.numTargets);
      if (result.pose.isPresent()){
        nt_pose_x.set(result.pose.get().estimatedPose.getX());
        nt_pose_y.set(result.pose.get().estimatedPose.getY());
        nt_pose_t.set(result.pose.get().estimatedPose.getRotation().toRotation2d().getRadians());
        nt_timestamp.set(result.pose.get().timestampSeconds);
      }
      if (result.bestTarget.isPresent()){
        nt_besttarget.set(result.bestTarget.get().getFiducialId());
        nt_ambiguity.set(result.bestTarget.get().getPoseAmbiguity());
      }
    }
  }

  /**
  * Reloads the AprilTag field layout when the field layout file changes
  */
  public void reloadFieldLayout() {
    try {
        // Update field layout path
        String fieldLayoutPath = VisionConstants.getFieldLayoutPath();
        
        // Load new field layout
        aprilTagFieldLayout = new AprilTagFieldLayout(fieldLayoutPath);
        
        // Update pose estimators with new field layout
        if (photonPoseEstimatorFront != null) {
            photonPoseEstimatorFront.setFieldTags(aprilTagFieldLayout);
        }
        if (photonPoseEstimatorRear != null) {
            photonPoseEstimatorRear.setFieldTags(aprilTagFieldLayout);
        }
        
        // Update simulation if in sim mode
        if (Robot.isSimulation() && simVision != null) {
            simVision.clearAprilTags();
            simVision.addAprilTags(aprilTagFieldLayout);
        }
    } catch (IOException e) {
        e.printStackTrace();
    }
  }
}
