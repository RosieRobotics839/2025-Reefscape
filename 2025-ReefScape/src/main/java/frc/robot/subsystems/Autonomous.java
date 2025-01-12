// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.utils.VectorUtils;
import frc.utils.pathfinding.astar.PathfindingUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Autonomous extends SubsystemBase {

  private static Autonomous instance = new Autonomous();
  public static Autonomous getInstance(){return instance;}

  List<Translation2d> m_pathpoints;

  public Pose2d m_aimPoint;
  public double m_aimPointRotationOffset;

  public static List<Translation2d> bluestage = new ArrayList<Translation2d>(){{
    add(new Translation2d(2.85,4.45));
    add(new Translation2d(2.85,3.00));
    add(new Translation2d(5.62, 1.45));
    add(new Translation2d(6.35, 2.25));
    add(new Translation2d(6.35, 5.85));
    add(new Translation2d(5.62, 6.50));
  }};
  //public static List<Translation2d> redstage = new ArrayList<Translation2d>(){{
  //  add(new Translation2d(13.75,4.45));
  //  add(new Translation2d(13.75,3.60));
  //  add(new Translation2d(11.0, 2));
  //  add(new Translation2d(10.35, 2.435));
  //  add(new Translation2d(10.35, 5.625));
  //  add(new Translation2d(11.0, 6.05));
  //}};
  //public static List<Translation2d> bluespeaker = new ArrayList<Translation2d>(){{
  //  add(new Translation2d(0,6.85));
  //  add(new Translation2d(1.112,6.175));
  //  add(new Translation2d(1.112, 4.724));
  //  add(new Translation2d(0, 4.084));
  //}};
  //public static List<Translation2d> redspeaker = new ArrayList<Translation2d>(){{
  //  add(new Translation2d(16.542,6.762));
  //  add(new Translation2d(15.464,6.14));
  //  add(new Translation2d(15.464, 4.672));
  //  add(new Translation2d(16.542, 4.119));
  //}};

  public static List<List<Translation2d>> staticObstacles = new ArrayList<List<Translation2d>>(){{
    double halffield = Vision.getInstance().aprilTagFieldLayout.getFieldLength()/2.0;
    
    List<Translation2d> redstage = bluestage.stream().map(f->new Translation2d(halffield+(halffield-f.getX()),f.getY())).collect(Collectors.toList());
    add(bluestage);
    add(redstage);
    
    //List<Translation2d> redspeaker = bluespeaker.stream().map(f->new Translation2d(halffield+(halffield-f.getX()),f.getY())).collect(Collectors.toList());
    //add(bluespeaker);
    //add(redspeaker);

    var it = iterator();
    Integer i=0;
    while(it.hasNext()){
      var fieldobj=PoseEstimator.getInstance().m_field.getObject("KeepOut"+(i++).toString());
      fieldobj.setPoses(it.next().stream().map(a->new Pose2d(a,new Rotation2d(0))).collect(Collectors.toList()));
    }    
  }};

  static NetworkTable table;
  final BooleanPublisher nt_instage;

  public void aimAtNote(){

    Vision.getInstance().findPixyTarget();
    
    double angleDiff = Vision.getInstance().m_pixyTargetAngle/AutoConstants.kaimNoteGain;
    double curAngle = PoseEstimator.getInstance().m_finalPose.getRotation().getRadians();

    DriveTrain.getInstance().setTargetHeading(curAngle + angleDiff*(Math.PI/180.0));
    // might need: DriveConstants.kAutoMaxRotAcceleration*(Vision.getInstance().frameCenterOffset /  + Math.signum(targetOffset) * 0.20));
  }

  public void aimAtPoint(Pose2d aimPoint){
    aimAtPoint(aimPoint, 0);
  }
  
  public void aimAtPoint(Pose2d aimPoint, double radiansOffset){
    m_aimPoint = aimPoint;
    m_aimPointRotationOffset = radiansOffset;
  }

  public void stopAiming(){
    m_aimPoint = null;
  }

  /** Creates a new Autonomous. */
  public Autonomous() {
    table = NetworkTableInstance.getDefault().getTable("roboRIO/Autonomous");
    nt_instage = table.getBooleanTopic("isInStage").publish();
  }

  List<AprilTag> allTags;
  public Rotation2d bestTagRotation(Pose2d pose){
    if (allTags == null){allTags = Vision.getInstance().aprilTagFieldLayout.getTags();}
    
    Rotation2d bestRotation = null;
    double bestRadius = 3;
    for (int i=0; i<allTags.size(); i++){
      Pose2d tagPose = allTags.get(i).pose.toPose2d();
      Pose2d diff = VectorUtils.poseDiff(pose,tagPose);
      double diffAngle = diff.getTranslation().getAngle().getRadians();
      double tagPoseAngle = tagPose.getRotation().getRadians();
      double angleFromTag = Math.abs(VectorUtils.angleDifference(diffAngle,tagPoseAngle));
      if (angleFromTag < Units.degreesToRadians(60)){
        if (diff.getTranslation().getNorm() < bestRadius){
          bestRadius = diff.getTranslation().getNorm();
          bestRotation = diff.getTranslation().getAngle().rotateBy(new Rotation2d(Math.PI));
        }
      }
    }
    return bestRotation;
  }

  @Override
  public void periodic() {
    
    // Look for best tag while traveling autonomously
    if (DriveConstants.kAutoTurnToBestTag && DriveTrain.getInstance().m_poseQueue.size() >= 1){
      Rotation2d bestDirection = bestTagRotation(PoseEstimator.getInstance().m_finalPose);
      if (bestDirection != null){
        DriveTrain.getInstance().setTargetHeading(bestDirection.getRadians());
      }
    }
    
    if (m_aimPoint != null){
      double heading = VectorUtils.vectorInDirectionOf(
        VectorUtils.poseDiff(m_aimPoint, PoseEstimator.getInstance().m_finalPose),1).getAngle().getRadians();
      DriveTrain.getInstance().setTargetHeading(heading + m_aimPointRotationOffset);
    }

    boolean insideStage = PathfindingUtils.PointInPolygon(PoseEstimator.getInstance().m_finalPose.getTranslation(), bluestage);
    nt_instage.set(insideStage);
  }
}
