package frc.robot.subsystems;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.CalibrationMap;
import frc.utils.Motor;
import frc.utils.NTValues.NTDouble;
import frc.robot.Constants.GameConstants;

public class Arm extends SubsystemBase{

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Arm/table");
    
    DoublePublisher nt_currentAngle, nt_targetAngle;

    private static Arm instance = new Arm(ArmConstants.kArmCANID, ArmConstants.kDigitalInputID);

    public static Arm getInstance(){
        return instance;
    }

    public Motor m_motorArm;
    public DutyCycleEncoder m_armAnalogEncoder;
    public double m_currentAngle;
    public double m_angleTarget = NTDouble.create(0, table, "targetAngle",(val)->setArmAngle(val));
    public double targetAngle = 0;
    public double AngleMin = ArmConstants.kAngleMin;
    public double AngleMax = ArmConstants.kAngleMax;
    public double elevatorCurrentHeight;
    double m_armOffset = 0; //change later
    double m_newArmOffset = 0;
    boolean m_setupArmDone = false;
    boolean scoringTrough = false;
    boolean scoringLevels2or3 = false;
    boolean scoringLevel4 = false;
    GameConstants.ScoreLevel m_scoreReefLevel;

    DoublePublisher
    nt_armOffset;

    // Checking to see if we are at the score position.
    public Boolean atScorePosition(){
        return Math.abs(m_currentAngle - m_angleTarget) < ArmConstants.kArmAngleTolerance; 
    }

    public void setArmAngleSafely(double target) {
        if (!m_setupArmDone) return;
        
        // Get current elevator position
        boolean inDangerZone = Elevator.getInstance().isInDangerZone();
        
        if (inDangerZone) {
            // If in danger zone, limit angle to safe value
            target = Math.min(target, GameConstants.kAngleMaxDZ);
        }
        
        // Apply normal min/max bounds
        target = Math.max(AngleMin, Math.min(AngleMax, target));
        m_motorArm.setPosition(target);
        m_angleTarget = target;
    }

    public void setArmAngle(double target) {
        setArmAngleSafely(target);
    }
    CalibrationMap m_armCalibrationMap = new CalibrationMap(ArmConstants.kArmCalibrationX, ArmConstants.kArmCalibrationY);
        
    public boolean calibrationValid(){
        return m_armAnalogEncoder.get() > m_armCalibrationMap.xmin() && m_armAnalogEncoder.get() < m_armCalibrationMap.xmax();
    }

    public Command Calibrate = Commands.sequence(
        Commands.waitUntil(this::calibrationValid),
        Commands.waitUntil(()->m_motorArm.isSetupDone()),
        Commands.waitUntil(()->m_motorArm.setEncoderPosition(m_armCalibrationMap.get(m_armAnalogEncoder.get())))
    );

    public Arm(int CANID, int analogID) {

        nt_armOffset = table.getDoubleTopic("angle/armOffset").publish();
        nt_currentAngle = table.getDoubleTopic("angle/currentAngle").publish();
        nt_targetAngle = table.getDoubleTopic("angle/currentAngle").publish();

        m_armAnalogEncoder = new DutyCycleEncoder(analogID);

        m_motorArm = new Motor(ArmConstants.kArmCANID, ArmConstants.kMotorType, "arm")
            .withStatorLimit((int)ArmConstants.kArmMotorCurrentLimit)
            .inverted(true)
            .withGearRatio(ArmConstants.kArmGearRatio)
            .withSpeedLimit(ArmConstants.kMaxSpeed)
            .setCalibration(m_newArmOffset*4096.0);

        Calibrate.schedule();
    } 

    public Command createMoveToAngleCommand(double target) {
        return Commands.sequence(
            // Check if movement is safe based on elevator position
            Commands.waitUntil(() -> {
                if (Elevator.getInstance().isInDangerZone()) {
                    targetAngle = Math.min(target, GameConstants.kAngleMaxDZ);
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
        if (m_motorArm.isSetupDone()){
            m_currentAngle = m_motorArm.getPosition();
        }

        setArmAngle(m_angleTarget);
        nt_armOffset.set(m_armAnalogEncoder.get());
        nt_currentAngle.set(m_currentAngle);
        nt_targetAngle.set(m_angleTarget);

    }
}
