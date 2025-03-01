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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoreConstants;
import frc.utils.Motor;
import frc.utils.Motor.GainSlot;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;
import frc.utils.NTValues.NTInteger;

public class Elevator extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Elevator");
    DoublePublisher nt_currentHeight, nt_targetHeight;
    BooleanPublisher nt_calibrated, nt_limitSwitch;

    private static Elevator instance = new Elevator();
    public static Elevator getInstance(){
        return instance;
    }

    public Motor m_EleMotor;
    boolean setupElevator = false;
    public double m_targetHeight = NTDouble.create(0, table, "targetHeight",(val)->setPosition(Units.inchesToMeters(val)));
    public double m_currentHeight = 0;
    ScoreConstants.ScoreLevel m_scoreReefLevel;
    public double armCurrentAngle;

    DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.klimitSwitchChannel);
    Trigger limitTrigger = new Trigger(() -> limitSwitch.get());

    /**
     * Sets the target elevator position. It may not go there due to arm and elevator safety protections.
     * @param position Target height in meters
     */
    public void setPosition(Double position) {
        m_targetHeight = position;
    }

    /**
     * Returns the current elevator height based on the left elevator motor position multiplied by the circumference of the sprocket.
     * @return Elevator distance in inches from bottom limit switch
     */
    public double getPosition() {
        return m_EleMotor.getPosition() * ElevatorConstants.kSprocketCircumference;
    }

    // Check if elevator is at target position
    public boolean isAtPosition() {
        return Math.abs(getPosition() - m_targetHeight) <= ElevatorConstants.kElevatorTolerance/ElevatorConstants.kSprocketCircumference; 
    }

    public boolean isInDangerZone() {
        Boolean inDZ = (getPosition() > ElevatorConstants.kLimitUnderDZ && getPosition() <= ElevatorConstants.kLimitAboveDZ);
        Boolean targetInDZ = (m_targetHeight > ElevatorConstants.kLimitUnderDZ && m_targetHeight <= ElevatorConstants.kLimitAboveDZ);
        Boolean crossingDZ = (getPosition() <= ElevatorConstants.kLimitUnderDZ && m_targetHeight >= ElevatorConstants.kLimitUnderDZ) 
                                || (getPosition() >= ElevatorConstants.kLimitAboveDZ && m_targetHeight <= ElevatorConstants.kLimitAboveDZ);
        return inDZ || targetInDZ || crossingDZ || !m_isCalibrated.get();
    }

    private void setElevatorHeightSafely(double safeTargetHeight) {
        // Check if we're about to enter danger zone
        if (Arm.getInstance().isInDangerZone()){
            // Use the average of the limits to decide if we are above or below the danger zone
            if (getPosition() > ((ElevatorConstants.kLimitUnderDZ + ElevatorConstants.kLimitAboveDZ) / 2)){
                safeTargetHeight = Math.min(Math.max(safeTargetHeight, ElevatorConstants.kLimitAboveDZ), ElevatorConstants.kMaxHeight);
            } else {
                safeTargetHeight = Math.min(Math.max(safeTargetHeight, ElevatorConstants.kMinHeight), ElevatorConstants.kLimitUnderDZ);
            }
        }
        
        // Ensure target height is within allowed range
        safeTargetHeight = Math.min(Math.max(safeTargetHeight, ElevatorConstants.kMinHeight), ElevatorConstants.kMaxHeight);
        
        // Convert inches to motor rotations and set position
        double targetRotations = safeTargetHeight / ElevatorConstants.kSprocketCircumference;
        m_EleMotor.setPosition(targetRotations);
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

        m_EleMotor = new Motor(ElevatorConstants.kEleCANID, ElevatorConstants.kMotorType, "elevator")
            .inverted(true)
            .withStatorLimit((int)ElevatorConstants.kElevatorMotorCurrentLimit)
            .withGearRatio(ElevatorConstants.kElevatorGearRatio)
            .withSpeedLimit(ElevatorConstants.kMaxSpeedPositive, ElevatorConstants.kMaxSpeedNegative)
            .withSlowSpeedControl(true)
            .pidf(ElevatorConstants.kGainPosition, GainSlot.POSITION);
    }

    public Command createMoveToHeightCommand(double targetHeight) {
        return Commands.sequence(
            // move elevator
            Commands.runOnce(() -> setPosition(targetHeight))
        );
    }

    // Example commands for different heights
    public Command moveToTroughCommand() {
        return createMoveToHeightCommand(ElevatorConstants.kHeight1);
    }

    public Command moveToLevel2Command() {
        return createMoveToHeightCommand(ElevatorConstants.kHeight2);
    }

    public Command moveToLevel3Command() {
        return createMoveToHeightCommand(ElevatorConstants.kHeight3);
    }

    public Command moveToLevel4Command() {
        return createMoveToHeightCommand(ElevatorConstants.kMaxHeight);
    }

    NTBoolean m_isCalibrated = new NTBoolean(false,table,"isCalibrated",(val)->{});
    NTInteger m_isCalibrating = new NTInteger(0,table,"isCalibrating",(val)->{if (val == -1){}});;
    
    private Command increment(){
        return new InstantCommand(()->{m_isCalibrating.set(m_isCalibrating.get()+1);});
    }

    Command c_calibrate = calibrate();
    private Command calibrate(){ return Commands.sequence(
            // Prep for calibration
            new InstantCommand(()->{
                m_isCalibrating.set(1);
                m_EleMotor
                    .withSpeedLimit(ElevatorConstants.kCalibrationSpeed/ElevatorConstants.kSprocketCircumference)
                    .stopMotor();
            }),
            // If limit switch is pressed, move up off of it.
            Commands.sequence(
                new InstantCommand(()->{
                    m_EleMotor
                        .setRelativePosition(ElevatorConstants.kCalibrationUpTravel/ElevatorConstants.kSprocketCircumference);
                }),
                Commands.waitUntil(()->!limitSwitch.get())
            ).unless(()->!limitSwitch.get()),
            // Move down until limit switch is hit
            new InstantCommand(()->m_EleMotor.setRelativePosition(-(ElevatorConstants.kMaxHeight-ElevatorConstants.kMinHeight)/ElevatorConstants.kSprocketCircumference)),
            Commands.waitUntil(()->{return limitSwitch.get();}).andThen(increment()).finallyDo(()->m_EleMotor.stopMotor()),
            // Move up for slow calibration
            new InstantCommand(()->m_EleMotor.setRelativePosition((ElevatorConstants.kMaxHeight-ElevatorConstants.kMinHeight)/ElevatorConstants.kSprocketCircumference)),
            Commands.waitUntil(()->!limitSwitch.get()).andThen(increment()).finallyDo(()->m_EleMotor.stopMotor()),
            // Move down until limit switch is hit again
            new InstantCommand(()->m_EleMotor
                .withSpeedLimit(ElevatorConstants.kCalibrationSlowSpeed/ElevatorConstants.kSprocketCircumference)
                .setRelativePosition(-(ElevatorConstants.kMaxHeight-ElevatorConstants.kMinHeight))),
            // When limit switch is pressed, stop the motor and zero out the relative encoders.
            Commands.waitUntil(()->limitSwitch.get()).finallyDo(()->{m_EleMotor.stopMotor();}).andThen(
                Commands.parallel(
                    Commands.waitUntil(()->m_EleMotor.setEncoderPosition(0))
                )
            ).andThen(increment()),
            new InstantCommand(()->{
                // End calibration and set the position target to 0.
                setPosition(0);
                m_isCalibrated.set(true);
                m_isCalibrating.set(0);
                // Reset Elevator Speed Limits
                m_EleMotor.withSpeedLimit(ElevatorConstants.kMaxSpeedPositive/ElevatorConstants.kSprocketCircumference, ElevatorConstants.kMaxSpeedNegative/ElevatorConstants.kSprocketCircumference);
            })
        ).finallyDo(()->{c_calibrate=calibrate();});
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled() || !m_isCalibrated.get()){
            m_targetHeight = getPosition();
        }

        // Start calibration sequence
        if (DriverStation.isEnabled()){
            if (!m_isCalibrated.get() && m_isCalibrating.get()==0){
                calibrate().schedule();
            }

            if (m_isCalibrated.get() && m_isCalibrating.get()==0){
                setElevatorHeightSafely(m_targetHeight);
            }
        }

        nt_currentHeight.set(Units.metersToInches(getPosition()));
        nt_targetHeight.set(Units.metersToInches(m_targetHeight));
        nt_limitSwitch.set(limitSwitch.get());

    }
}
