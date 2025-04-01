package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.EffectorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoreConstants;
import frc.robot.Constants.ScoreConstants.ScoreLevel;
import frc.robot.Constants.VisionConstants;
import frc.utils.VectorUtils;

public class AutoCommands {

    static public Command noop(){return new InstantCommand(()->{});}; //noop

    public static boolean contains(ArrayList<Integer> array, int value) {
        for (int element : array) {
            if (element == value) {
                return true; // Value found
            }
        }
        return false; // Value not found
    }

    // Helper functions for april tag selection on blue vs red alliance.
    static public int coralSourceLTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 13 : 1 ); }
    static public int coralSourceRTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 12 : 2 ); }
    static public int bargeFrontTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 14 : 5 ); }
    static public int bargeBackTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 4 : 15 ); }
    static public int processorTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 3 : 16 ); }
    // Direction of reef tags are in cardinal direction (North, South, ect.) from perspective of respective team driver station
    static public int reefNWTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 20 : 11 ); }
    static public int reefNNTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 21 : 10 ); }
    static public int reefNETag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 22 : 9 ); }
    static public int reefSWTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 19 : 6 ); }
    static public int reefSSTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 18 : 7 ); }
    static public int reefSETag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 17 : 8 ); }

    static public ArrayList <Integer> reefIDs(){ return new ArrayList<Integer>(Arrays.asList(reefNWTag(), reefNNTag(), reefNETag(), reefSWTag(), reefSSTag(), reefSETag()));};
    static public ArrayList <AprilTag> reefTags(){ return Vision.getInstance().aprilTagFieldLayout.getTags().stream().filter(m->contains(reefIDs(),m.ID)).collect(Collectors.toCollection(ArrayList::new));};
    static public ArrayList <Pose2d> reefPoses(){ return reefTags().stream().map(m->m.pose.toPose2d()).collect(Collectors.toCollection(ArrayList::new));};

    static public ArrayList <Integer> sourceIDs(){ return new ArrayList<Integer>(Arrays.asList(coralSourceLTag(), coralSourceRTag()));};
    static public ArrayList <AprilTag> sourceTags(){ return Vision.getInstance().aprilTagFieldLayout.getTags().stream().filter(m->contains(sourceIDs(),m.ID)).collect(Collectors.toCollection(ArrayList::new));};
    static public ArrayList <Pose2d> sourcePoses(){ return sourceTags().stream().map(m->m.pose.toPose2d()).collect(Collectors.toCollection(ArrayList::new));};

    public static Command NavToPose(Pose2d pose){
        return new InstantCommand(() -> PathPlanning.getInstance().navigateTo(pose));
    }

    static Pose2d m_storedPose;
    public static Command StorePose(){
        return new InstantCommand(() -> m_storedPose = PoseEstimator.getInstance().m_finalPose);
    }

    public static Command ReturnToPose(){
        return new InstantCommand(() -> {
                if (m_storedPose != null){
                    PathPlanning.getInstance().navigateTo(m_storedPose);
                }
            }
        );
    }

    /*
    public static Command GetCoral(){ // "if anybody fixes the spelling i am NOT quitting programming :)" - Dean.
                                      // "Either you run the day, or the day runs you" - Dean.
                                      // "Change your thoughts and you change your world" - Dean.
                                      // I don't think i said that - Dean.
        ArrayList<Integer> tagIds = new ArrayList<Integer>();
        { 
            tagIds.add(coralSourceRTag());
            tagIds.add(coralSourceLTag());
        }
        
        ArrayList<Pose2d> tags = tagIds.stream().map(id->Vision.getInstance().aprilTagFieldLayout.getTagPose(id).get().toPose2d()).collect(Collectors.toCollection(ArrayList::new));
        
        Pose2d target = PoseEstimator.getInstance().m_finalPose.nearest(tags);
        int tagId = tagIds.get(tags.indexOf(target));
        return GetCoral(tagId);
    }
    */

    public static Command GetCoral(boolean left){
        return GetCoral(left ? coralSourceLTag() : coralSourceRTag());
    }
    
    public static Command GetCoral(int tagId){
        return Commands.parallel(
            Commands.sequence(
                Commands.waitUntil(() -> !Autonomous.getInstance().isInsideReef()),
                new InstantCommand(() -> {
                    Elevator.getInstance().setPosition(0);
                    Arm.getInstance().setPosition(ArmConstants.kAngleMax);
                })),
            Commands.sequence(
                new InstantCommand(() -> {
                    PathPlanning.getInstance().navigateTo(PathPlanning.AprilTagAtDistance(tagId, -AutoConstants.kSourceStartingDistance - Constants.kChassis.kWheelBase/2.0, Math.PI));
                }),
                new InstantCommand(() -> PathPlanning.getInstance().navigateTo(PathPlanning.AprilTagAtDistance(tagId, -AutoConstants.kSourceDistance - Constants.kChassis.kWheelBase/2.0, Math.PI))),
                Commands.waitUntil(() -> VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose,PathPlanning.AprilTagAtDistance(tagId, -AutoConstants.kSourceStartingDistance - Constants.kChassis.kWheelBase/2.0), AutoConstants.kSourceTolerance)),
                Commands.parallel(
                    EndEffector.getInstance().IntakeCommand().asProxy(),
                    wiggle.asProxy()
                ).withTimeout(5)
            ).repeatedly().handleInterrupt(()->DriveTrain.getInstance().m_poseQueue.clear())
        ).until(()->{return !Autonomous.getInstance().isInsideReef() && EndEffector.getInstance().hasGamePiece();});
    }

    public static Command wiggle = Commands.sequence(
        Commands.waitSeconds(0.1),
        new InstantCommand(()->DriveTrain.getInstance().m_targetHeading += Units.degreesToRadians(10)),
        Commands.waitSeconds(0.1),
        new InstantCommand(()->DriveTrain.getInstance().m_targetHeading -= Units.degreesToRadians(20)),
        Commands.waitSeconds(0.1),
        new InstantCommand(()->DriveTrain.getInstance().m_targetHeading += Units.degreesToRadians(10))
    ).repeatedly().until(()->EndEffector.getInstance().hasGamePiece());

    public static Command Score(ScoreConstants.ScoreLevel level){
        return Commands.sequence(
            new InstantCommand(() -> Elevator.getInstance().moveToLevelCommand(()->level)),
            new InstantCommand(() -> Arm.getInstance().moveToLevelCommand(()->level)),
            Controller.accessoryButtons.m_expel.asProxy()
        );
    }

    public static Command AutoScore(String tag, boolean left, ScoreLevel level){
        int tagId;
        switch (tag){
            case("NW"):
                tagId = reefNWTag(); 
                break;
            case("NN"):
                tagId = reefNNTag(); 
                break;
            case("NE"):
                tagId = reefNETag(); 
                break;
            case("SW"):
                tagId = reefSWTag(); 
                break;
            case("SS"):
                tagId = reefSSTag(); 
                break;
            case("SE"):
                tagId = reefSETag(); 
                break;
            default:
                return noop();
        }

        Pose2d target1 = PathPlanning.AprilTagAtDistance(
            tagId,
            new Translation2d(
                - AutoConstants.kReefStartingDistance - Constants.kChassis.kWheelBase/2.0,
                Constants.AutoConstants.kReefOffset * (left ? 1 : -1)+ Constants.AutoConstants.kStaticReefOffset
            ),Units.degreesToRadians(15)
        );

        Pose2d target2 = PathPlanning.AprilTagAtDistance(
            tagId,
            new Translation2d(
                - AutoConstants.kReefDistance - Constants.kChassis.kWheelBase/2.0,
                Constants.AutoConstants.kReefOffset * (left ? 1 : -1) + Constants.AutoConstants.kStaticReefOffset
            )
        );

        return Commands.sequence(
            new InstantCommand(()->{Autonomous.getInstance().m_drivingToReef = true;
                                    Autonomous.getInstance().aimAtPoint(Autonomous.reefCenter(), VisionConstants.frontCamera.kCamYawRight);
                                    PathPlanning.getInstance().navigateTo(target1);
                                }),
            new InstantCommand(() -> PathPlanning.getInstance().navigateTo(target2)),
            Commands.waitUntil(() -> DriveTrain.getInstance().m_poseQueue.isEmpty() || DriveTrain.getInstance().m_isStoppedConfirmed || VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose, target1, AutoConstants.kReefArmupTolerance)).withTimeout(3),
            new InstantCommand(() -> Elevator.getInstance().moveToLevel(level)),
            Commands.sequence(new InstantCommand(() -> Arm.getInstance().moveToLevel(level))),
            new InstantCommand(() -> Autonomous.getInstance().stopAiming()),
            Commands.waitUntil(() -> DriveTrain.getInstance().m_poseQueue.isEmpty() || DriveTrain.getInstance().m_isStoppedConfirmed || VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose, target2, AutoConstants.kReefTolerance)).withTimeout(6),
            Commands.waitUntil(() -> Arm.getInstance().isAtPosition() && Elevator.getInstance().isAtPosition()),
            EndEffector.getInstance().ExpelCommand(()->(level == ScoreLevel.TROUGH ? EffectorConstants.kTroughOuttakeSpeed : EffectorConstants.kOuttakeSpeed), ()->level==ScoreLevel.TROUGH).withTimeout(1.5)
        );
    }

    public static Command BargeFling(){
        return Commands.sequence(
            new InstantCommand(() -> Elevator.getInstance().setPosition(ElevatorConstants.kMaxHeight)),
            new InstantCommand(() -> Arm.getInstance().setPosition(ArmConstants.kAngleMin)),
            Commands.waitUntil(() -> Arm.getInstance().isAtPosition()),
            Commands.waitUntil(() -> Elevator.getInstance().isAtPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> Arm.getInstance().setPosition(ArmConstants.kAngleMax)),
            Commands.waitUntil(() -> Arm.getInstance().isNearPosition()),
            new InstantCommand(() -> EndEffector.getInstance().JustShootIt())
        );
    }
    
    public static void DriveReefOffset() {
        Pose2d target = PoseEstimator.getInstance().m_finalPose.nearest(reefPoses());
    
        // Calculate lateral offset based on alignment selection
        Translation2d approachOffset;
        Translation2d finalOffset;
    
        switch (Controller.m_reefAlign) {
            case CENTER:
                // Center alignment - no lateral offset
                approachOffset = new Translation2d(
                    -AutoConstants.kReefStartingDistance - Constants.kChassis.kWheelBase/2.0,
                    Constants.AutoConstants.kStaticReefOffset
                );
    
                finalOffset = new Translation2d(
                    -AutoConstants.kReefDistance - Constants.kChassis.kWheelBase/2.0,
                    Constants.AutoConstants.kStaticReefOffset
                );
                break;
    
            case RIGHT:
                // Right alignment - negative offset
                approachOffset = new Translation2d(
                    -AutoConstants.kReefStartingDistance - Constants.kChassis.kWheelBase/2.0,
                    -Constants.AutoConstants.kReefOffset + Constants.AutoConstants.kStaticReefOffset
                );
    
                finalOffset = new Translation2d(
                    -AutoConstants.kReefDistance - Constants.kChassis.kWheelBase/2.0,
                    -Constants.AutoConstants.kReefOffset + Constants.AutoConstants.kStaticReefOffset
                );
                break;
    
            case LEFT:
            default:
                // Left alignment - positive offset
                approachOffset = new Translation2d(
                    -AutoConstants.kReefStartingDistance - Constants.kChassis.kWheelBase/2.0,
                    Constants.AutoConstants.kReefOffset + Constants.AutoConstants.kStaticReefOffset
                );
    
                finalOffset = new Translation2d(
                    -AutoConstants.kReefDistance - Constants.kChassis.kWheelBase/2.0,
                    Constants.AutoConstants.kReefOffset + Constants.AutoConstants.kStaticReefOffset
                );
                break;
        }
        
        // Create target poses
        Pose2d target1 = PathPlanning.PoseAtDistance(target, approachOffset, Units.degreesToRadians(15));
        Pose2d target2 = PathPlanning.PoseAtDistance(target, finalOffset);

        PathPlanning.getInstance().navigateTo(target1);
        PathPlanning.getInstance().navigateTo(target2);
        Autonomous.getInstance().m_drivingToReef = true;

    }

    public static void AimAtClosestSource(){
        Pose2d target = PoseEstimator.getInstance().m_finalPose.nearest(sourcePoses());
        DriveTrain.getInstance().lockTargetHeading(target.getRotation().getRadians());
    }
}