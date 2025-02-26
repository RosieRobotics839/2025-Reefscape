package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
    BooleanPublisher nt_calibrated, nt_limitSwitch;

    private static Elevator instance = new Elevator();
    public static Elevator getInstance(){
        return instance;
    }

    public Motor m_EleMotorLeft;
    public Motor m_EleMotorRight;
    boolean setupElevator = false;
    public double m_targetHeight = NTDouble.create(0, table, "targetHeight",(val)->setPosition(Units.inchesToMeters(val)));
    public double m_currentHeight = 0;
    GameConstants.ScoreLevel m_scoreReefLevel;
    public double minHeightInch = ElevatorConstants.kMinHeightInch;
    public double maxHeightInch = ElevatorConstants.kMaxHeightInch;
    public double armCurrentAngle;

    DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.klimitSwitchChannel);
    Trigger limitTrigger = new Trigger(() -> limitSwitch.get());

    public void setPosition(Double position) {
        setPosition(position);
    }

    /**
     * Returns the current elevator height based on the left elevator motor position multiplied by the circumference of the sprocket.
     * @return Elevator distance in inches from bottom limit switch
     */
    public double getPosition() {
        return m_EleMotorLeft.getPosition() * (Math.PI * ElevatorConstants.kSprocketDiameter);
    }

    // Check if elevator is at target position
    public boolean isAtPosition() {
        return Math.abs(getPosition() - m_targetHeight) < ElevatorConstants.kElevatorTolerance; 
    }

    public boolean isInDangerZone() {
        return (getPosition() > ElevatorConstants.kLimitUnderDZ && 
                getPosition() <= ElevatorConstants.kLimitAboveDZ);
    }

    public void setElevatorHeightSafely(double safeTargetHeight) {
        // Check if we're about to enter danger zone
        if (Arm.getInstance().isInDangerZone()){
            // Use the average of the limits to decide if we are above or below the danger zone
            if (getPosition() > ((ElevatorConstants.kLimitUnderDZ + ElevatorConstants.kLimitAboveDZ) / 2)){
                safeTargetHeight = Math.min(Math.max(safeTargetHeight, ElevatorConstants.kLimitAboveDZ), maxHeightInch);
            } else {
                safeTargetHeight = Math.min(Math.max(safeTargetHeight, minHeightInch), ElevatorConstants.kLimitUnderDZ);
            }
        }
        
        // Ensure target height is within allowed range
        safeTargetHeight = Math.min(Math.max(safeTargetHeight, minHeightInch), maxHeightInch);
        
        // Convert inches to motor rotations and set position
        double targetRotations = safeTargetHeight / (Math.PI * ElevatorConstants.kSprocketDiameter);
        m_EleMotorLeft.setPosition(targetRotations);
    }

    /**
     * Sets the elevator target height
     * @param target (meters)
     */
    public void setPosition(double target) {
        m_targetHeight = target;
    }

    public Elevator (){

        nt_currentHeight = table.getDoubleTopic("currentHeight").publish();
        nt_targetHeight = table.getDoubleTopic("targetHeight").publish();
        nt_calibrated = table.getBooleanTopic("calibrated").publish();
        nt_limitSwitch = table.getBooleanTopic("limitSwitch").publish();
        
        m_EleMotorLeft = new Motor(ElevatorConstants.kEleLeftCANID, ElevatorConstants.kMotorType, "eleLeft")
            .inverted(false)
            .withStatorLimit((int)ElevatorConstants.kLeftElevatorMotorCurrentLimit)
            .withSpeedLimit(ElevatorConstants.kMaxSpeedPositive,ElevatorConstants.kMaxSpeedNegative)
            .withGearRatio(ElevatorConstants.kElevatorGearRatio)
            .withSpeedLimit(ElevatorConstants.kMaxSpeedPositive, ElevatorConstants.kMaxSpeedNegative)
            .pidf(ElevatorConstants.kGainPosition, GainSlot.POSITION);

        m_EleMotorRight = new Motor(ElevatorConstants.kEleRightCANID, ElevatorConstants.kMotorType, "eleRight")
            .inverted(true)
            .withStatorLimit((int)ElevatorConstants.kRightElevatorMotorCurrentLimit)
            .withGearRatio(ElevatorConstants.kElevatorGearRatio)
            .withSpeedLimit(ElevatorConstants.kMaxSpeedPositive, ElevatorConstants.kMaxSpeedNegative)
            .pidf(ElevatorConstants.kGainPosition, GainSlot.POSITION)
            .setFollowerMode(ElevatorConstants.kEleLeftCANID, false); // not sure if we need to invert motor second time, test this

    }

    public Command createMoveToHeightCommand(double targetHeight) {
        return Commands.sequence(
            // move elevator
            Commands.runOnce(() -> setPosition(targetHeight))
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
    boolean m_lastLimitSwitch = false;

    @Override
    public void periodic() {

        if (DriverStation.isDisabled() || !m_isCalibrated){
            m_targetHeight = getPosition();
        }

        // Handle limit switch calibration either disabled or enabled. Requires the limit switch to transition from not pressed to pressed for now.
        if (!m_lastLimitSwitch && limitSwitch.get() && !m_isCalibrated){
            m_EleMotorLeft.setEncoderPosition(0); // Reset encoder at bottom limit
            m_EleMotorRight.setEncoderPosition(0);
            m_targetHeight = Units.inchesToMeters(0);
            m_isCalibrated = true;
            m_lastLimitSwitch = true;
        }

        if (DriverStation.isEnabled()){
            if (m_isCalibrated){
                setElevatorHeightSafely(m_targetHeight);
            }
        }

        nt_calibrated.set(m_isCalibrated);
        nt_currentHeight.set(Units.metersToInches(getPosition()));
        nt_targetHeight.set(Units.metersToInches(m_targetHeight));
        nt_limitSwitch.set(limitSwitch.get());

    }
}
