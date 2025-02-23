package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.Calibrate;
import frc.utils.Motor;
import frc.utils.NTValues.NTDouble;
import frc.robot.Constants.GameConstants;

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
    public double m_angleTarget = NTDouble.create(0, table, "angle/targetAngle",(val)->setArmAngle(val));
    public double targetAngle = 0;
    public double AngleMin = ArmConstants.kAngleMin;
    public double AngleMax = ArmConstants.kAngleMax;
    public double elevatorCurrentHeight;
    double m_armOffset = 0; //change later
    double m_newArmOffset = 0;
    boolean m_setupDone = false;
    boolean scoringTrough = false;
    boolean scoringLevels2or3 = false;
    boolean scoringLevel4 = false;
    GameConstants.ScoreLevel m_scoreReefLevel;

    DoublePublisher
        nt_positionSensor,
        nt_setupDone,
        nt_safetyLimit,
        nt_motorCommand;

    // Checking to see if we are at the score position.
    public Boolean atScorePosition(){
        return Math.abs(m_currentAngle - m_angleTarget) < ArmConstants.kAngleTolerance; 
    }

    public void setArmAngleSafely(double target) {
        nt_setupDone.set(m_setupDone ? 1.0 : 0.0);
        if (!m_setupDone){
            System.out.println("Arm setup not done, ignoring command");
            return;
        }
        
        // Get current elevator position
        boolean inDangerZone = Elevator.getInstance().isInDangerZone();
        
        if (inDangerZone) {
            // If in danger zone, limit angle to safe value
            target = Math.min(target, ArmConstants.kAngleMaxDZ);
        }
        
        // Apply normal min/max bounds
        target = Math.max(AngleMin, Math.min(AngleMax, target));

        // Debug output for safety limits
        nt_safetyLimit.set(target);

        // Convert degrees to rotations and send to motor
        double motorCommand = target/(2.0*Math.PI);
        nt_motorCommand.set(motorCommand);
        m_motor.setPosition(motorCommand);
        m_angleTarget = target;
    }

    public void setArmAngle(double target) {
        setArmAngleSafely(target);
    }

    public Arm(int CANID, int analogID) {

        nt_positionSensor = table.getDoubleTopic("angle/positionSensor").publish();
        nt_currentAngle = table.getDoubleTopic("angle/currentAngle").publish();
        nt_targetAngle = table.getDoubleTopic("angle/targetAngle").publish();
        nt_setupDone = table.getDoubleTopic("debug/setupDone").publish();
        nt_safetyLimit = table.getDoubleTopic("debug/safetyLimit").publish();
        nt_motorCommand = table.getDoubleTopic("debug/motorCommand").publish();

        m_angleSensor = new DutyCycleEncoder(analogID);

        m_motor = new Motor(ArmConstants.kCANID, ArmConstants.kMotorType, "arm")
            .withStatorLimit((int)ArmConstants.kArmMotorCurrentLimit)
            .inverted(true)
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

    public Command createMoveToAngleCommand(double target) {
        return Commands.sequence(
            // Check if movement is safe based on elevator position
            Commands.waitUntil(() -> {
                if (Elevator.getInstance().isInDangerZone()) {
                    targetAngle = Math.min(target, ArmConstants.kAngleMaxDZ);
                }
                return true;
            }),
            // Move to safe angle
            Commands.runOnce(() -> setArmAngle(targetAngle)),
            // Wait until movement is complete
            Commands.waitUntil(this::atScorePosition)
        );
    }

    // Example commands for different positions
    public Command moveToTroughCommand() {
        return createMoveToAngleCommand(ArmConstants.kTargetAngleTrough);
    }

    public Command moveToLevel2Command() {
        return createMoveToAngleCommand(ArmConstants.kTargetAngleLevelMiddle);
    }

    public Command moveToLevel3Command() {
        return createMoveToAngleCommand(ArmConstants.kTargetAngleLevelMiddle);
    }

    public Command moveToLevel4Command() {
        return createMoveToAngleCommand(ArmConstants.kTargetAngleLevel4);
    }

    @Override
    public void periodic() {

        // Update current Angle from encoder position
        if (m_motor.isSetupDone()){
            m_currentAngle = m_motor.getPosition();
        }

        setArmAngle(m_angleTarget);
        nt_positionSensor.set(m_angleSensor.get());
        nt_currentAngle.set(m_currentAngle);
        nt_targetAngle.set(m_angleTarget);

    }
}
