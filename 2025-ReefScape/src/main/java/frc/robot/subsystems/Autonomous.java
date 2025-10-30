// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTBoolean;
import frc.utils.pathfinding.astar.PathfindingUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Autonomous extends SubsystemBase {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Autonomous");

  private static Autonomous instance = new Autonomous();
  public static Autonomous getInstance(){return instance;}

  List<Translation2d> m_pathpoints;

  public Boolean m_drivingToReef = false;
  public Boolean m_drivingToBarge = false;
  public Pose2d m_aimPoint;
  public double m_aimPointRotationOffset;
  private NTBoolean nt_isInsideReef = new NTBoolean(false,table,"isInsideReef",null);
  private Debouncer m_insideReefDebounce = new Debouncer(0.7777777, DebounceType.kFalling);

  public static Pose2d m_redReefCenter = PathPlanning.AprilTagAtDistance(AutoConstants.kReefRedCenterRefID, AutoConstants.kFieldReefCenterFromAprilTagDistance);
  public static Pose2d m_blueReefCenter = PathPlanning.AprilTagAtDistance(AutoConstants.kReefBlueCenterRefID, AutoConstants.kFieldReefCenterFromAprilTagDistance);
  
  public static Pose2d reefCenter(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? Autonomous.m_blueReefCenter : Autonomous.m_redReefCenter ); }
  public static List<Translation2d> reefObstacle(){ return(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? Autonomous.bluereef : Autonomous.redreef ); }

  public static List<Translation2d> generateReefKOZ(Pose2d center){
    return new ArrayList<Translation2d>(){{
      for (int i=0; i<6; i++){
        add(center.getTranslation().plus(
          new Translation2d(
            AutoConstants.kReefKOZRadius,
            new Rotation2d(center.getRotation().getRadians()+Units.degreesToRadians(30+60*i))
          )
        ));
      }
    }};
  }
  
  public static List<Translation2d> redreef = generateReefKOZ(m_redReefCenter);
  public static List<Translation2d> bluereef = generateReefKOZ(m_blueReefCenter);
  
  public static List<Translation2d> bargecolumn = new ArrayList<Translation2d>(){{
    add(new Translation2d(8.160,3.46));
    add(new Translation2d(8.160,4.475));
    add(new Translation2d(9.5, 4.475));
    add(new Translation2d(9.5, 3.46));
  }};

  public static List<Translation2d> flipRedToBlue(List<Translation2d> obstacle){
    double halffield = Vision.getInstance().aprilTagFieldLayout.getFieldLength()/2.0;
    return obstacle.stream().map(f->new Translation2d(halffield-(f.getX()-halffield),f.getY())).collect(Collectors.toList());
  }
  
  public static List<List<Translation2d>> staticObstacles = generateStaticObstacles();
  public static List<List<Translation2d>> generateStaticObstacles(){
    Autonomous.redreef = Autonomous.generateReefKOZ(m_redReefCenter);
    Autonomous.bluereef = Autonomous.generateReefKOZ(m_blueReefCenter);
    return new ArrayList<List<Translation2d>>(){{
      add(bluereef);
      add(redreef);
      add(bargecolumn);
  
      publishObstacles(this);
    }};
  }

  public static void publishObstacles(List<List<Translation2d>> list){
    var it = list.iterator();
    Integer i=0;
    while(it.hasNext()){
      var fieldobj=PoseEstimator.getInstance().m_field.getObject("KeepOut"+(i++).toString());
      fieldobj.setPoses(it.next().stream().map(a->new Pose2d(a,new Rotation2d(0))).collect(Collectors.toList()));
    } 
  }
  
  public void aimAtPoint(Translation2d aimPoint){
    aimAtPoint(new Pose2d(aimPoint, new Rotation2d(0)), 0);
  }

  public void aimAtPoint(Translation2d aimPoint, double radiansOffset){
    aimAtPoint(new Pose2d(aimPoint, new Rotation2d(0)), radiansOffset);
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

  public boolean isInsideReef(){
    return nt_isInsideReef.get();
  }

  @Override
  public void periodic() {
    
    if (DriveTrain.getInstance().m_poseQueue.isEmpty()){
      m_drivingToReef = false;
      m_drivingToBarge = false;
    }

    nt_isInsideReef.set(m_insideReefDebounce.calculate(PathfindingUtils.PointInConvexPolygon(PoseEstimator.getInstance().m_finalPose.getTranslation(), reefObstacle())));

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

  }
}
