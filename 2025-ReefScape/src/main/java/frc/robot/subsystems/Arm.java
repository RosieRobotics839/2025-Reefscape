package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.Calibrate;
import frc.utils.Motor;
import frc.utils.NTValues.NTDouble;
import frc.robot.Constants.ScoreConstants.ScoreLevel;

public class Arm extends SubsystemBase{

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Arm/table");
    
    DoublePublisher nt_currentAngle, nt_targetAngle;

    private static Arm instance = new Arm(ArmConstants.kCANID, ArmConstants.kDigitalInputID);

    public static Arm getInstance(){
        return instance;
    }

    public Motor m_motor;
    public DutyCycleEncoder m_angleSensor;
    public double m_currentAngle = 0;
    public double m_angleTarget = Units.degreesToRadians(NTDouble.create(90, table, "angle/targetAngle",(val)->setPosition(Units.degreesToRadians(val))));
    public double elevatorCurrentHeight;
    double m_armOffset = 0; //change later
    double m_newArmOffset = 0;
    boolean m_setupDone = false;
    boolean scoringTrough = false;
    boolean scoringLevels2or3 = false;
    boolean scoringLevel4 = false;

    DoublePublisher
        nt_positionSensor,
        nt_safetyLimit,
        nt_motorCommand;

    BooleanPublisher
        nt_setupDone;

    // Checking to see if we are at the score position.
    public Boolean isAtPosition(){
        return Math.abs(getArmPosition() - m_angleTarget) < ArmConstants.kAngleTolerance; 
    }

    /**
     * Returns arm position in radians
     * @return Arm Position
     */
    public double getArmPosition(){
        return m_motor.getPosition() * (2 * Math.PI);
    }

    private void setArmAngleSafely(double target) {
        if (!m_setupDone){
            return;
        }
        
        // Get current elevator position
        boolean inDangerZone = Elevator.getInstance().isInDangerZone();
        
        if (inDangerZone) {
            // If in danger zone, limit angle to safe value
            target = Math.min(target, ArmConstants.kAngleMaxDZ-ArmConstants.kAngleDZMargin);
        }
        
        // Apply normal min/max bounds
        target = Math.max(ArmConstants.kAngleMin, Math.min(ArmConstants.kAngleMax, target));

        // Debug output for safety limits
        nt_safetyLimit.set(Units.radiansToDegrees(target));

        // Convert degrees to rotations and send to motor
        double motorCommand = target/(2.0*Math.PI);
        nt_motorCommand.set(motorCommand);
        m_motor.setPosition(motorCommand);
    }

    public boolean isInDangerZone(){
        return getArmPosition() > ArmConstants.kAngleMaxDZ;
    }

    /**
     * Sets the arm position, protected by safe limits
     * @param radians target position
     */
    public void setPosition(double radians){
        m_angleTarget = radians;
    }

    public Arm(int CANID, int analogID) {

        nt_positionSensor = table.getDoubleTopic("angle/positionSensor").publish();
        nt_currentAngle = table.getDoubleTopic("angle/currentAngle").publish();
        nt_targetAngle = table.getDoubleTopic("angle/targetAngle").publish();
        nt_setupDone = table.getBooleanTopic("debug/setupDone").publish();
        nt_safetyLimit = table.getDoubleTopic("debug/safetyLimit").publish();
        nt_motorCommand = table.getDoubleTopic("debug/motorCommand").publish();

        m_angleSensor = new DutyCycleEncoder(analogID);

        m_motor = new Motor(ArmConstants.kCANID, ArmConstants.kMotorType, "arm")
            .withStatorLimit((int)ArmConstants.kArmMotorCurrentLimit)
            .inverted(false)
            .withGearRatio(ArmConstants.kArmGearRatio)
            .withSpeedLimit(ArmConstants.kMaxSpeed);

        Calibrate.motor("arm",
            ArmConstants.kCalibrationX,
            ArmConstants.kCalibrationY,
            m_motor,
            ()->m_angleSensor.get(),
            new InstantCommand(()->m_setupDone = true)
        );
    } 

    public void moveToLevel(ScoreLevel level){
        switch (level){
            case FUNNEL:
                setPosition(ArmConstants.kAngleMax);
                break;
            case TROUGH:
                setPosition(ArmConstants.kTargetAngleTrough);
                break;
            case LEVEL2:
                setPosition(ArmConstants.kTargetAngleLevelMiddle);
                break;
            case LEVEL3:
                setPosition(ArmConstants.kTargetAngleLevelMiddle);
                break;
            case LEVEL4:
                setPosition(ArmConstants.kTargetAngleLevel4);
                break;
            case ALGAE:
                setPosition(ArmConstants.kAngleMin);
                break;
            default:
        }
    }
    public Command moveToLevelCommand(Supplier<ScoreLevel> level) {
        return new InstantCommand(()->moveToLevel(level.get()))
        .unless(()->EndEffector.getInstance().m_intakeRunning);
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()){
            m_angleTarget = getArmPosition();
        }

        setArmAngleSafely(m_angleTarget);

        nt_setupDone.set(m_setupDone);
        nt_positionSensor.set(m_angleSensor.get());
        nt_currentAngle.set(Units.radiansToDegrees(getArmPosition()));
        nt_targetAngle.set(Units.radiansToDegrees(m_angleTarget));

    }
}
