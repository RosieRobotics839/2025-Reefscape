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
        return Math.abs(m_currentHeight - m_targetHeight) < ElevatorConstants.kElevatorTolerance; 
    }

    public boolean isInDangerZone() {
        return (m_currentHeight > GameConstants.kLimitUnderDZ && 
                m_currentHeight <= GameConstants.kLimitAboveDZ);
    }

    public void setElevatorHeightSafely(double targetHeightInches) {
        // Check if we're about to enter danger zone
        boolean willBeInDangerZone = (targetHeightInches > GameConstants.kLimitUnderDZ && 
                                     targetHeightInches <= GameConstants.kLimitAboveDZ);
        
        if (willBeInDangerZone) {
            // If arm angle is too large, adjust arm first
            double currentArmAngle = Arm.getInstance().m_currentAngle;
            if (currentArmAngle > GameConstants.kAngleMaxDZ) {
                Arm.getInstance().setArmAngleSafely(GameConstants.kAngleMaxDZ);
                // Wait for arm to reach safe position before moving elevator
                if (!Arm.getInstance().atScorePosition()) {
                    return;
                }
            }
        }
        
        // Ensure target height is within allowed range
        m_targetHeight = Math.min(Math.max(targetHeightInches, minHeightInch), maxHeightInch);
        
        // Convert inches to motor rotations and set position
        double targetRotations = m_targetHeight / (2 * Math.PI); // need to scale this
        m_EleMotorLeft.setPosition(targetRotations);
    }

    public void setElevatorHeight(double targetHeightInches) {
        setElevatorHeightSafely(targetHeightInches);
    }

    public Elevator (){

        nt_currentHeight = table.getDoubleTopic("currentHeight").publish();
        nt_targetHeight = table.getDoubleTopic("targetHeight").publish();
        
        m_EleMotorLeft = new Motor(ElevatorConstants.kEleLeftCANID, ElevatorConstants.kMotorType, "eleLeft")
            .inverted(false)
            .withStatorLimit((int)ElevatorConstants.kLeftElevatorMotorCurrentLimit)
            .withSpeedLimit(ElevatorConstants.kMaxSpeedPositive,ElevatorConstants.kMaxSpeedNegative)
            .withGearRatio(ElevatorConstants.kElavatorGearRatio)
            .withKI(ElevatorConstants.kElevatorKi,GainSlot.POSITION);

        m_EleMotorRight = new Motor(ElevatorConstants.kEleRightCANID, ElevatorConstants.kMotorType, "eleRight")
            .inverted(true)
            .withStatorLimit((int)ElevatorConstants.kRightElevatorMotorCurrentLimit)
            .withGearRatio(ElevatorConstants.kElavatorGearRatio)
            .withKI(ElevatorConstants.kElevatorKi,GainSlot.POSITION)
            .setFollowerMode(ElevatorConstants.kEleLeftCANID, false); // not sure if we need to invert motor second time, test this

    }

    public Command createMoveToHeightCommand(double targetHeight) {
        return Commands.sequence(
            // First check if we're moving into danger zone
            Commands.waitUntil(() -> {
                if (targetHeight > GameConstants.kLimitUnderDZ && 
                    targetHeight <= GameConstants.kLimitAboveDZ) {
                    // If moving into danger zone, ensure arm is safe first
                    double currentArmAngle = Arm.getInstance().m_currentAngle;
                    if (currentArmAngle > GameConstants.kAngleMaxDZ) {
                        return false; // Wait for arm to be safe, unsure if this will get stuck forever
                    }
                }
                return true;
            }),
            // Now safe to move elevator
            Commands.runOnce(() -> setElevatorHeight(targetHeight)),
            // Wait until movement is complete
            Commands.waitUntil(this::isAtPosition)
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

    @Override
    public void periodic() {

        // Update current height from encoder position, we only need to check the leader motor
        if (m_EleMotorLeft.isSetupDone()) {
            m_currentHeight = m_EleMotorLeft.getPosition() * (2 * Math.PI); // Convert motor rotations to inches
        }

        // Stop at limit switch
        if (limitSwitch.get()) {
            m_currentHeight = ElevatorConstants.kMinHeightInch;
            m_EleMotorLeft.setPosition(0); // Reset encoder at bottom limit
            m_EleMotorRight.setPosition(0);
        }
        
        setElevatorHeight(m_targetHeight);
        nt_currentHeight.set(m_currentHeight);
        nt_targetHeight.set(m_targetHeight);
    }
}
