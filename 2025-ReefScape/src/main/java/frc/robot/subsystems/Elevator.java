package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Motor;


public class Elevator {

    private static Elevator instance = new Elevator();

    public static Elevator getInstance(){
        return instance;
    }

    public Motor m_EleMotorLeft;
    public Motor m_EleMotorRight;
    boolean setupElevator = false;

    DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.klimitSwitchChanel);
    Trigger limitTrigger = new Trigger(() -> limitSwitch.get()); 

    public double getElevatorHeight(){
        return 0.0;
    }

    public void setElevatorHeight(double target){

    }

    public Elevator (){
        
        m_EleMotorLeft = new Motor(ElevatorConstants.kEleLeftCANID, ElevatorConstants.kMotorType, "eleLeft")
            .inverted(false)
            .smartCurrentLimit(ElevatorConstants.kLeftElevatorMotorCurrentLimit)
            .pidf(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd, ElevatorConstants.kElevatorKff);

        m_EleMotorRight = new Motor(ElevatorConstants.kEleRightCANID, ElevatorConstants.kMotorType, "eleRight")
            .inverted(true)
            .smartCurrentLimit(ElevatorConstants.kRightElevatorMotorCurrentLimit)
            .pidf(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd, ElevatorConstants.kElevatorKff);
    }
}
