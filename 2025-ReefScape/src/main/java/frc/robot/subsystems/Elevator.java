package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameConstants;
import frc.utils.Motor;

public class Elevator extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Elevator");
    DoublePublisher nt_currentHeight, nt_targetHeight;

    private static Elevator instance = new Elevator();
    public static Elevator getInstance(){
        return instance;
    }

    public Motor m_EleMotorLeft;
    public Motor m_EleMotorRight;
    boolean setupElevator = false;
    public double m_targetHeight = 0;
    public double m_currentHeight = 0;
    GameConstants.ScoreLevel m_scoreReefLevel;
    public double minHeightInch = ElevatorConstants.kMinHeightInch;
    public double maxHeightInch = ElevatorConstants.kMaxHeightInch;
    public double armCurrentAngle;

    Command ElevatorCommand = Commands.sequence(
        Commands.waitUntil(() -> {
            switch(m_scoreReefLevel){
                case TROUGH:
                    m_targetHeight = ElevatorConstants.kHeight1Inch;
                case LEVEL2:
                    m_targetHeight = ElevatorConstants.kHeight2Inch;
                case LEVEL3:
                    m_targetHeight = ElevatorConstants.kHeight3Inch;
                case LEVEL4:
                    m_targetHeight = ElevatorConstants.kMaxHeightInch;
            }
            return false;
        }),
        Commands.waitUntil(() -> {return isAtPosition();})
    );

    DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.klimitSwitchChannel);
    Trigger limitTrigger = new Trigger(() -> limitSwitch.get());

    public void setPosition(Double position) {
        setElevatorHeight(position);
    }

    public double getElevatorHeight() {
        return m_currentHeight;
    }

    public void setElevatorHeight(double targetHeightInches){
        // ensure target height is within allowed range
        m_targetHeight = Math.min(Math.max(targetHeightInches, minHeightInch), maxHeightInch);

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

        // Sets variable to be instance of arm class
        armCurrentAngle = Arm.getInstance().m_currentAngle;

        // Checking to see if the current angle is greater than or equal to the init minimum angle of the arm and less than or equal to the init maximum angle of the arm.
        // Once true it sets the min height of the elevator to be the init minimum height and the max height to be the limit under the danger zone.
        if (armCurrentAngle >= ArmConstants.kAngleMin && armCurrentAngle <= ArmConstants.kAngleMax){
            minHeightInch = ElevatorConstants.kMinHeightInch;
            maxHeightInch = GameConstants.kLimitUnderDZ;
        // Checking to see if the current height is greater than or equal to the limit above the danger zone and less than or equal to the init maximum height.
        // Once true it sets the minimum height to be the limit above the danger zone and the max height to be the init maximum height.
        // Doing this to differentiate whether the elevator is already above the danger zone or not.
        } else if (m_currentHeight >= GameConstants.kLimitAboveDZ && m_currentHeight <= ElevatorConstants.kMaxHeightInch){
            minHeightInch = GameConstants.kLimitAboveDZ;
            maxHeightInch = ElevatorConstants.kMaxHeightInch;
        }

        // Stop at limit
        if (limitSwitch.get()) {
            m_currentHeight = ElevatorConstants.kMinHeightInch;
            m_EleMotorLeft.setPosition(0); // Reset encoder at bottom limit
            m_EleMotorRight.setPosition(0);
        }
        
        setElevatorHeight(m_targetHeight);
    }
}
