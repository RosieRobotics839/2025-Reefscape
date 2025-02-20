package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Motor;
import frc.utils.Motor.MyMotorType;

public class Elevator extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Elevator");
    DoublePublisher nt_currentHeight, nt_targetHeight;

    private static Elevator instance = new Elevator();
    public static Elevator getInstance(){
        return instance;
    }

    private double m_targetHeight = ElevatorConstants.kMinHeightInch;
    private double m_currentHeight = ElevatorConstants.kMinHeightInch;

    public static enum Position {
        LEVEL4(ElevatorConstants.kMaxHeightInch),
        LEVEL3(ElevatorConstants.kHeight3Inch),
        LEVEL2(ElevatorConstants.kHeight2Inch),
        TROUGH(ElevatorConstants.kHeight1Inch),
        MIN(ElevatorConstants.kMinHeightInch);

        public final double height;
        Position(double height) {
            this.height = height;
        }
    }

    Command ElevatorCommand = Commands.sequence(
        Commands.waitUntil(() -> {
            switch(m_scoreReefLevel){
                case TROUGH:

                case LEVEL2:

                case LEVEL3:

                case LEVEL4:
            }
            return false;
        }),
        Commands.waitUntil(() -> {return isAtPosition();})
    );

    public Motor m_EleMotorLeft;
    public Motor m_EleMotorRight;
    boolean setupElevator = false;
    GameConstants.ScoreLevel m_scoreReefLevel;

    DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.klimitSwitchChanel);
    Trigger limitTrigger = new Trigger(() -> limitSwitch.get());

    public void setPosition(Position position) {
        setElevatorHeight(position.height);
    }

    public double getElevatorHeight() {
        return m_currentHeight;
    }

    public void setElevatorHeight(double targetHeightInches){
        // ensure target height is within allowed range
        m_targetHeight = Math.min(Math.max(targetHeightInches, ElevatorConstants.kMinHeightInch), ElevatorConstants.kMaxHeightInch);

        // Convert inches to motor rotations and set position
        double targetRotations = m_targetHeight / (2 * Math.PI); // We need to adjust this conversion factor to elevator motor gearing
        m_EleMotorLeft.setPosition(targetRotations);

    }

    // Check if elevator is at target position
    public boolean isAtPosition() {
        return Math.abs(m_currentHeight - m_targetHeight) < ElevatorConstants.kElevatorTolerance; 
    }

    public Elevator (){

        nt_currentHeight = table.getDoubleTopic("currentHeight").publish();
        nt_targetHeight = table.getDoubleTopic("targetHeight").publish();
        
        m_EleMotorLeft = new Motor(ElevatorConstants.kEleLeftCANID, ElevatorConstants.kMotorType, "eleLeft")
            .inverted(false)
            .smartCurrentLimit(ElevatorConstants.kLeftElevatorMotorCurrentLimit)
            .pidf(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd, ElevatorConstants.kElevatorKff);

        m_EleMotorRight = new Motor(ElevatorConstants.kEleRightCANID, ElevatorConstants.kMotorType, "eleRight")
            .inverted(true)
            .smartCurrentLimit(ElevatorConstants.kRightElevatorMotorCurrentLimit)
            .pidf(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd, ElevatorConstants.kElevatorKff)
            .setFollowerMode(ElevatorConstants.kEleLeftCANID, false); // not sure if we need to invert motor second time, test this

    }

    @Override
    public void periodic() {

        nt_currentHeight.set(m_currentHeight);
        nt_targetHeight.set(m_targetHeight);

        // Update current height from encoder position, we only need to check the leader motor
        if (m_EleMotorLeft.isSetupDone()) {
            m_currentHeight = m_EleMotorLeft.getPosition() * (2 * Math.PI); // Convert motor rotations to inches
        }

        // Stop at limit
        if (limitSwitch.get()) {
            m_currentHeight = ElevatorConstants.kMinHeightInch;
            m_EleMotorLeft.setPosition(0); // Reset encoder at bottom limit
            m_EleMotorRight.setPosition(0);
        }
    }
}
