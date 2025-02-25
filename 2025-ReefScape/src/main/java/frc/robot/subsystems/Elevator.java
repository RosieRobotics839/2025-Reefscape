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
import frc.robot.Constants.GameConstants;
import frc.utils.Motor;
import frc.utils.Motor.GainSlot;
import frc.utils.NTValues.NTDouble;

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
    public double m_targetHeight = NTDouble.create(0, table, "targetHeight",(val)->setElevatorHeight(val));
    public double m_currentHeight = 0;
    GameConstants.ScoreLevel m_scoreReefLevel;
    public double minHeightInch = ElevatorConstants.kMinHeightInch;
    public double maxHeightInch = ElevatorConstants.kMaxHeightInch;
    public double armCurrentAngle;

    DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.klimitSwitchChannel);
    Trigger limitTrigger = new Trigger(() -> limitSwitch.get());

    public void setPosition(Double position) {
        setElevatorHeight(position);
    }

    public double getElevatorHeight() {
        return m_currentHeight;
    }

    // Check if elevator is at target position
    public boolean isAtPosition() {
        return Math.abs(getElevatorHeight() - m_targetHeight) < ElevatorConstants.kElevatorTolerance; 
    }

    public boolean isInDangerZone() {
        return (getElevatorHeight() > ElevatorConstants.kLimitUnderDZ && 
                getElevatorHeight() <= ElevatorConstants.kLimitAboveDZ);
    }

    public void setElevatorHeightSafely(double targetHeight) {
        // Check if we're about to enter danger zone
        if (Arm.getInstance().isInDangerZone()){
            // Use the average of the limits to decide if we are above or below the danger zone
            if (getElevatorHeight() > ((ElevatorConstants.kLimitUnderDZ + ElevatorConstants.kLimitAboveDZ) / 2)){
                targetHeight = Math.min(Math.max(targetHeight, ElevatorConstants.kLimitAboveDZ), maxHeightInch);
            } else {
                targetHeight = Math.min(Math.max(targetHeight, minHeightInch), ElevatorConstants.kLimitUnderDZ);
            }
        }
        
        // Ensure target height is within allowed range
        targetHeight = Math.min(Math.max(targetHeight, minHeightInch), maxHeightInch);
        
        // Convert inches to motor rotations and set position
        double targetRotations = m_targetHeight / (Math.PI * ElevatorConstants.kSprocketDiameter);
        m_EleMotorLeft.setPosition(targetRotations);
    }

    /**
     * Sets the elevator target height
     * @param target (meters)
     */
    public void setElevatorHeight(double target) {
        m_targetHeight = target;
    }

    public Elevator (){

        nt_currentHeight = table.getDoubleTopic("currentHeight").publish();
        nt_targetHeight = table.getDoubleTopic("targetHeight").publish();
        
        m_EleMotorLeft = new Motor(ElevatorConstants.kEleLeftCANID, ElevatorConstants.kMotorType, "eleLeft")
            .inverted(false)
            .withStatorLimit((int)ElevatorConstants.kLeftElevatorMotorCurrentLimit)
            .withSpeedLimit(ElevatorConstants.kMaxSpeedPositive,ElevatorConstants.kMaxSpeedNegative)
            .withGearRatio(ElevatorConstants.kElevatorGearRatio)
            .pidf(ElevatorConstants.kGainPosition, GainSlot.POSITION);

        m_EleMotorRight = new Motor(ElevatorConstants.kEleRightCANID, ElevatorConstants.kMotorType, "eleRight")
            .inverted(true)
            .withStatorLimit((int)ElevatorConstants.kRightElevatorMotorCurrentLimit)
            .withGearRatio(ElevatorConstants.kElevatorGearRatio)
            .pidf(ElevatorConstants.kGainPosition, GainSlot.POSITION)
            .setFollowerMode(ElevatorConstants.kEleLeftCANID, false); // not sure if we need to invert motor second time, test this

    }

    public Command createMoveToHeightCommand(double targetHeight) {
        return Commands.sequence(
            // move elevator
            Commands.runOnce(() -> setElevatorHeight(targetHeight))
        );
    }

    // Example commands for different heights
    public Command moveToTroughCommand() {
        return createMoveToHeightCommand(ElevatorConstants.kHeight1Inch);
    }

    public Command moveToLevel2Command() {
        return createMoveToHeightCommand(ElevatorConstants.kHeight2Inch);
    }

    public Command moveToLevel3Command() {
        return createMoveToHeightCommand(ElevatorConstants.kHeight3Inch);
    }

    public Command moveToLevel4Command() {
        return createMoveToHeightCommand(ElevatorConstants.kMaxHeightInch);
    }
    boolean m_isCalibrated = false;
    @Override
    public void periodic() {

        // Stop at limit switch
        if (!m_isCalibrated && limitSwitch.get()) {
            m_EleMotorLeft.setEncoderPosition(0); // Reset encoder at bottom limit
            m_EleMotorRight.setEncoderPosition(0);
            m_isCalibrated = true;
        }
        
        setElevatorHeightSafely(m_targetHeight);
        nt_currentHeight.set(m_currentHeight);
        nt_targetHeight.set(m_targetHeight);
    }
}
