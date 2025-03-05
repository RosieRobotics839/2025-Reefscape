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
    

    private class Argument{
        private int type;
        private int var;

        public Argument(String arg){
            arg = arg.toUpperCase();
            if (arg.contains("GET") || arg.contains("NOTE")){
                if (arg.contains("CEN") || arg.contains("TER")){type = 1;} // Center note type ID
                else type = 2; // Alliance notes type ID
            } else if (arg.contains("SPE") || arg.contains("KER")){
                type = 3; // Speaker Score type ID
            } else if (arg.contains("AM") || arg.contains("MP")){
                type = 4; // Amp Score type ID
            } else if (arg.contains("WAI") || arg.contains("AIT")){
                type = 5; // Wait type ID
            } else if (arg.contains("RET") || arg.contains("URN")){
                type = 6; // Return to pose type ID
            } else if (arg.contains("STO") || arg.contains("ORE")){
                type = 7; // Store pose type ID
            } else type = -1;
            try {var = Integer.parseInt(arg.substring(arg.length()-1)); } // Set var to hopefully the note ID
                catch (NumberFormatException e) {var = -1;}
                catch (StringIndexOutOfBoundsException e) {var = -1;}
        }

        public Command getCommand(){
            // if (!isReal()) return Commands.waitSeconds(0);
            switch (type){
                case(7):
                    return AutoCommands.StorePose();
                case(6):
                    return AutoCommands.ReturnToPose();
                case(5):
                    return Commands.waitSeconds(var);
                case(4):

                case(3):

                case(2):

                case(1):

                default:
                    return AutoCommands.noop(); //noop noop
            }
        }

    }
}