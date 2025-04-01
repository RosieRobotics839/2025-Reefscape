package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AutoCommands;

public class Dashboard{

    static Dashboard instance = new Dashboard();
    public static Dashboard getInstance(){
        return instance;
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
            INVALID, SCORE, GETCORAL, WAIT, POSERETURN, POSESTORE
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
            else type = TASK.INVALID;
            try { 
                if (input.contains("LEF") || input.contains("EFT")) left = true;
                else if (input.contains("RIG") || input.contains("GHT")) left = false;
                var0 = Integer.parseInt(input.substring(input.length()-1)); // Set var to hopefully the score level or wait time
                var1 = input.substring(input.indexOf(".")+1,input.indexOf(".")+3);
            } // Set var to hopefully the note ID
                catch (NumberFormatException e) {var0 = -1;}
                catch (StringIndexOutOfBoundsException e) {var0 = -1;}

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
                case SCORE:
                    // limit scoring level to trough through reef level 4
                    var level = Constants.ScoreConstants.ScoreLevel.values()[Math.max(1,Math.min(4,var0))];
                    return AutoCommands.AutoScore(var1,left,level);
                default:
                    return AutoCommands.noop(); //noop noop
            }
        }

    }
}