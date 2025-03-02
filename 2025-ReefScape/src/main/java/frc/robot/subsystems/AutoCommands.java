package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.utils.VectorUtils;

public class AutoCommands {
    static public Command noop(){return new InstantCommand(()->{});};

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

    public static Command GetCorral(){ // Only written for the leftmost coral source, and all values need to be thouroughly checked and tested
        return Commands.sequence(
            new InstantCommand(() -> Elevator.getInstance().setPosition(0)),
            new InstantCommand(() -> Arm.getInstance().setPosition(0)),
            new InstantCommand(() -> Autonomous.getInstance().aimAtPoint(PathPlanning.AprilTagAtDistance(coralSourceLTag(), 0),Units.degreesToRadians(180))),
            new InstantCommand(() -> PathPlanning.getInstance().navigateTo(new Pose2d(
                                        PathPlanning.AprilTagAtDistance( coralSourceLTag(), AutoConstants.kSourceLDistance).getTranslation(),
                                        new Rotation2d(Units.degreesToRadians(270))
                                    ))
                                ),
            Commands.waitUntil(() -> VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose,PathPlanning.AprilTagAtDistance(coralSourceLTag(),0),Units.feetToMeters(2),Math.PI))
        );
    }
    
    public static Command Score(double heightin, double anglerad){
        return Commands.sequence(
            new InstantCommand(() -> Elevator.getInstance().setPosition(heightin)),
            new InstantCommand(() -> Arm.getInstance().setPosition(anglerad)),
            // TODO: Fix this expel command
            EndEffector.getInstance().ExpelCommand(()->2.0,()->false),
            new InstantCommand(() -> Elevator.getInstance().setPosition(Constants.ElevatorConstants.kMinHeight)),
            new InstantCommand(() -> Arm.getInstance().setPosition(0))
        );
    }

}