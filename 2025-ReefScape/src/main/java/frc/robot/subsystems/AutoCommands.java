package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.utils.VectorUtils;

public class AutoCommands {
    static public Command noop(){return new InstantCommand(()->{});};

    // Helper functions for april tag selection on blue vs red alliance.
    //static public int speakerTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4 ); }
    //static public int ampTag(){     return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5 ); }
    //static public int sourceLTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 2 : 9 ); }
    //static public int sourceRTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 1 : 10); }

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
    
}