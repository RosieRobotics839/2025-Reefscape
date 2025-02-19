package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.utils.Motor;

public class Climber extends SubsystemBase{
    
    private static Climber instance = new Climber(ClimberConstants.kClimberCANID, ClimberConstants.kAnalogInputID);

    public static Climber getInstance(){
        return instance;
    }

    public Motor m_motorClimber;
    public boolean hasClimbed;

    public boolean hasClimbed(){ // Idea is to use this method to determine whether we have climbed or not, which will allow us to compress the climber command sequences to one command sequence as we have done before.
        return hasClimbed;
    }

    Command ClimberIN = Commands.sequence(
        Commands.waitUntil(() -> { m_motorClimber.setSpeed(-ClimberConstants.kClimberSpeed);}),
    );

    public Climber(int CANID, int analogID) {

        m_motorClimber = new Motor(ClimberConstants.kClimberCANID, ClimberConstants.kMotorType, "climber")
            .smartCurrentLimit((int)ClimberConstants.kClimberMotorCurrentLimit)
            .inverted(true)
            .positionConversionFactor((ClimberConstants.kClimberEncoderPositionFactor))
            //.setCalibration(m_newClimberOffset)
            .pidf(ClimberConstants.kClimberKp, ClimberConstants.kClimberKi, ClimberConstants.kClimberKd, ClimberConstants.kClimberKff);

    }

    @Override
    public void periodic() {
    
    }
}
