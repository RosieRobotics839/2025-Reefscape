package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ScoreConstants.ScoreLevel;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.robot.subsystems.AutoCommands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PathPlanning;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;
import frc.utils.VectorUtils;

public class Dashboard{

    static Dashboard instance = new Dashboard();
    public static Dashboard getInstance(){
        return instance;
    }

    int m_numAutoPoses = 6;
    List<Pose2d> m_autoPoses = new ArrayList<Pose2d>();
    {
        var horiz_margin=0.2;
        var vert_margin = 1.1;
        for (int i=0; i<m_numAutoPoses; i++){
        m_autoPoses.add(new Pose2d(Vision.getInstance().aprilTagFieldLayout.getFieldLength()*(horiz_margin+(i/(double)m_numAutoPoses)*(1-2*horiz_margin)),Vision.getInstance().aprilTagFieldLayout.getFieldWidth()*vert_margin, new Rotation2d(Math.PI/4)));
        }
        PoseEstimator.getInstance().m_field.getObject("AutoPoses").setPoses(m_autoPoses);
    }

    Argument arg0 = new Argument("DB/String 0");
    Argument arg1 = new Argument("DB/String 1");
    Argument arg2 = new Argument("DB/String 2");
    Argument arg3 = new Argument("DB/String 3");
    Argument arg4 = new Argument("DB/String 4");
    Argument arg5 = new Argument("DB/String 5");
    Argument arg6 = new Argument("DB/String 6");
    Argument arg7 = new Argument("DB/String 7");
    Argument arg8 = new Argument("DB/String 8");
    Argument arg9 = new Argument("DB/String 9");

    public Command BuildYourOwnAutoCommands(){
        m_autoPoses = PoseEstimator.getInstance().m_field.getObject("AutoPoses").getPoses();
        Command BuildYourOwnAutoCommands = Commands.sequence(
            arg0.getCommand(),
            arg1.getCommand(),
            arg2.getCommand(),
            arg3.getCommand(),
            arg4.getCommand(),
            arg5.getCommand(),
            arg6.getCommand(),
            arg7.getCommand(),
            arg8.getCommand(),
            arg9.getCommand());
        return BuildYourOwnAutoCommands;
    }
    
/*
 * SCORE command to drive to the reef and score should look like this
 * Score.SS.Left.4     The period are not required but help seperate the arguments for accessibility
 * ENSURE THAT THE SCORING LEVEL IS THE LAST CHAR IN THE STRING
 * 0 = trough, 4 = highest level
 * 
 * GET or SOURCE command to drive to the source to get a coral
 * Get.left || Source.right
 * 
 * WAIT command to wait a time in integer seconds
 * Wait.3
 * ENSURE THAT THE WAIT TIME IS THE LAST CHAR IN THE STRING
 * 
 * STORE command and RETURN command to store and return to a stored pose
 * Store
 * Return
 * Neither have any extra arguments
 */
    private class Argument{
        public enum TASK { 
            INVALID, SCORE, GETCORAL, WAIT, POSERETURN, POSESTORE, AUTOPOSE, GETALGAE, WAYPOINT, FLING
        }
        private TASK type;
        private int var0;
        private String var1;
        private boolean left = true;

        private String path;

        public Argument(String string){
            path = string;
        }

        public Command getCommand(){
            
            String input = SmartDashboard.getString(path, "null").toUpperCase();
            if (input.contains("SCO") || input.contains("CORE")) type = TASK.SCORE; // Scoring ID
            else if (input.contains("GET") || input.contains("SOURCE")) type = TASK.GETCORAL; // Get Coral ID
            else if (input.contains("WAI") || input.contains("AIT")) type = TASK.WAIT; // Wait type ID
            else if (input.contains("RET") || input.contains("URN")) type = TASK.POSERETURN; // Return to pose type ID
            else if (input.contains("STO") || input.contains("TOR")) type = TASK.POSESTORE; // Store pose type ID
            else if (input.contains("AUTOP") || input.contains("TOPOS")) type = TASK.AUTOPOSE;
            else if (input.contains("WAYP") || input.contains("YPOI")) type = TASK.WAYPOINT;
            else if (input.contains("ALG")) type = TASK.GETALGAE;
            else if (input.contains("FLI") || input.contains("ING")) type = TASK.FLING;
            else type = TASK.INVALID;
            if (input.contains("LEF") || input.contains("EFT"))
                left = true;
            else if (input.contains("RIG") || input.contains("GHT"))
                left = false;
            try { 
                var0 = Integer.parseInt(input.substring(input.length()-1)); // Set var to hopefully the score level or wait time
            } // Set var to hopefully the note ID
            catch (NumberFormatException e) {var0 = -1;}
            catch (StringIndexOutOfBoundsException e) {var0 = -1;}

            try {
                var1 = input.substring(input.indexOf(".")+1,input.indexOf(".")+3);
            }
            catch (StringIndexOutOfBoundsException e) {var1 = "";}

            // if (!isReal()) return Commands.waitSeconds(0);
            switch (type){
                case POSESTORE:
                    return AutoCommands.StorePose();
                case POSERETURN:
                    return AutoCommands.ReturnToPose();
                case WAIT:
                    return Commands.waitSeconds(var0);
                case GETCORAL:
                    return AutoCommands.GetCoral(left);
                case WAYPOINT:
                    return new InstantCommand(()->{
                        if (var0 >= 0 && var0 < m_autoPoses.size() && Vision.getInstance().poseIsInField(m_autoPoses.get(var0))){
                            PathPlanning.getInstance().navigateTo(m_autoPoses.get(var0));
                        }
                    }); 
                case AUTOPOSE:
                    return Commands.sequence(
                        new InstantCommand(()->{
                            if (var0 >= 0 && var0 < m_autoPoses.size() && Vision.getInstance().poseIsInField(m_autoPoses.get(var0))){
                                PathPlanning.getInstance().navigateTo(m_autoPoses.get(var0));
                            }
                        }),
                        Commands.waitUntil(()->{return DriveTrain.getInstance().m_poseQueue.isEmpty() || VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose, m_autoPoses.get(var0), DriveConstants.kAutoToleranceDistance);}).withTimeout(5)
                    );
                case SCORE:
                    // limit scoring level to trough through reef level 4
                    var level = Constants.ScoreConstants.ScoreLevel.values()[Math.max(1,Math.min(4,var0))];
                    return AutoCommands.AutoScore(var1,left,level);
                case GETALGAE:
                    switch (var0){
                    case 2:
                        return AutoCommands.GetAlgae(var1, ScoreLevel.ALGAE2);
                    case 3:
                        return AutoCommands.GetAlgae(var1, ScoreLevel.ALGAE3);
                    default:
                    case -1:
                        return AutoCommands.GetAlgae(var1);
                    }
                case FLING:
                    return Commands.sequence(
                            new InstantCommand(()->AutoCommands.DriveBargeOffset(Units.feetToMeters(var0), false)),
                            Commands.waitUntil(()->DriveTrain.getInstance().m_poseQueue.isEmpty()),
                            //new InstantCommand(() ->Elevator.getInstance().setPosition(ElevatorConstants.kMaxHeight)),
                            AutoCommands.BargeFling()
                    );
                default:
                    return AutoCommands.noop(); //noop noop
            }
        }

    }
}