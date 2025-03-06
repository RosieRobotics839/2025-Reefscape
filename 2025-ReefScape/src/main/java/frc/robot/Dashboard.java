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

    Argument arg0 = new Argument(SmartDashboard.getString("DB/String 0", "null"));
    Argument arg1 = new Argument(SmartDashboard.getString("DB/String 1", "null"));
    Argument arg2 = new Argument(SmartDashboard.getString("DB/String 2", "null"));
    Argument arg3 = new Argument(SmartDashboard.getString("DB/String 3", "null"));
    Argument arg4 = new Argument(SmartDashboard.getString("DB/String 4", "null"));
    Argument arg5 = new Argument(SmartDashboard.getString("DB/String 5", "null"));
    Argument arg6 = new Argument(SmartDashboard.getString("DB/String 6", "null"));
    Argument arg7 = new Argument(SmartDashboard.getString("DB/String 7", "null"));
    Argument arg8 = new Argument(SmartDashboard.getString("DB/String 8", "null"));
    Argument arg9 = new Argument(SmartDashboard.getString("DB/String 9", "null"));

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
        private int type;
        private int var0;
        private String var1;
        private boolean left = true;

        public Argument(String input){
            input = input.toUpperCase();
            if (input.contains("SCO") || input.contains("ORE")) type = 1; // Scoring ID
            else if (input.contains("GET") || input.contains("SOURCE")) type = 2; // Get Coral ID
            else if (input.contains("WAI") || input.contains("AIT")) type = 5; // Wait type ID
            else if (input.contains("RET") || input.contains("URN")) type = 6; // Return to pose type ID
            else if (input.contains("STO") || input.contains("ORE")) type = 7; // Store pose type ID
            else type = -1;
            try { 
                if (input.contains("LEF") || input.contains("EFT")) left = true;
                else if (input.contains("RIG") || input.contains("GHT")) left = false;
                var0 = Integer.parseInt(input.substring(input.length()-1)); // Set var to hopefully the score level or wait time
                var1 = input.substring(input.indexOf("."),input.indexOf(".")+2);
            } // Set var to hopefully the note ID
                catch (NumberFormatException e) {var0 = -1;}
                catch (StringIndexOutOfBoundsException e) {var0 = -1;}
        }

        public Command getCommand(){
            // if (!isReal()) return Commands.waitSeconds(0);
            switch (type){
                case(7):
                    return AutoCommands.StorePose();
                case(6):
                    return AutoCommands.ReturnToPose();
                case(5):
                    return Commands.waitSeconds(var0);
                case(2):
                    return AutoCommands.GetCoral(left);
                case(1):
                    return AutoCommands.AutoScore(var1,left,var0);
                default:
                    return AutoCommands.noop(); //noop noop
            }
        }

    }
}