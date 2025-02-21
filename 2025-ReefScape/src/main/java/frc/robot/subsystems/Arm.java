package frc.robot.subsystems;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.CalibrationMap;
import frc.utils.Motor;
import frc.robot.Constants.GameConstants;

public class Arm extends SubsystemBase{

    private static Arm instance = new Arm(ArmConstants.kArmCANID, ArmConstants.kAnalogInputID);

    public static Arm getInstance(){
        return instance;
    }

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Arm/table");
    
    public Motor m_motorArm;
    public AnalogInput m_armAnalogEncoder;
    public double m_currentAngle;
    public double m_angleTarget;
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

    /**
     * This function sets the Arm Angle to be within the target bounds of the minimum and maximum values which are being defined in Constants.
     * @param target Target angle in radians
     */
    public void setArmAngle(double target) {
        if (!m_setupArmDone) return;
        target = Math.max(AngleMin, Math.min(AngleMax, target));
        m_motorArm.setPosition(target);
        m_angleTarget = target;
    }

    // Checking to see if we are at the score position.
    public Boolean atScorePosition(){
        return Math.abs(m_currentAngle - m_angleTarget) < ArmConstants.kArmAngleTolerance; 
    }

    Command ArmPositionCommand = Commands.sequence(
        Commands.waitUntil(() -> {
            switch(m_scoreReefLevel){
                case TROUGH:
                    return m_motorArm.setPosition(ArmConstants.kTargetAngleTrough);
                case LEVEL2:
                case LEVEL3:
                    return m_motorArm.setPosition(ArmConstants.kTargetAngleLevelMiddle);
                case LEVEL4:
                    return m_motorArm.setPosition(ArmConstants.kTargetAngleLevel4);
            }
            return false;
        }),
        Commands.waitUntil(() -> {return atScorePosition();})
    );

    public Arm(int CANID, int analogID) {

        nt_armOffset = table.getDoubleTopic("angle/armOffset").publish();

        CalibrationMap m_armCalibrationMap = new CalibrationMap(ArmConstants.kArmCalibrationX, ArmConstants.kArmCalibrationY);
        
        m_armAnalogEncoder = new AnalogInput(analogID);
        m_armOffset = m_armAnalogEncoder.getValue();
        nt_armOffset.set(m_armOffset);
        if (m_armOffset > m_armCalibrationMap.xmin() && m_armOffset < m_armCalibrationMap.xmax()) {
            m_newArmOffset = m_armOffset;
            }

        m_motorArm = new Motor(ArmConstants.kArmCANID, ArmConstants.kMotorType, "arm")
            .smartCurrentLimit((int)ArmConstants.kArmMotorCurrentLimit)
            .inverted(true)
            .positionConversionFactor((ArmConstants.kArmEncoderPositionFactor))
            .setCalibration(m_newArmOffset)
            .pidf(ArmConstants.kArmKp, ArmConstants.kArmKi, ArmConstants.kArmKd, ArmConstants.kArmKff);

    } 

    @Override
    public void periodic() {
        
        // Update current Angle from encoder position
        if (m_motorArm.isSetupDone()){
            m_currentAngle = m_motorArm.getPosition();
        }

        elevatorCurrentHeight = Elevator.getInstance().m_currentHeight; // Setting variable to Instance of Elevator

        // Checking to see if the current height of the elevator is greater than the limit under the danger zone and less than or equal to the limit above the danger zone
        // After this check is done, and it returns true, it sets the angle max to be the max angle before the arm collides with the elevator and the min angle to the init mininum angle.
        if (elevatorCurrentHeight > GameConstants.kLimitUnderDZ && elevatorCurrentHeight <= GameConstants.kLimitAboveDZ){
            AngleMax = GameConstants.kAngleMaxDZ;
            AngleMin = ArmConstants.kAngleMin;
        // Checking to see if the current height of the elevator is either:
        // 1. Greater than or equal to the init minimum height of the elevator and less than or equal to the limit under the danger zone
        // 2. Greater than the limit above the danger zone and less than or equal to the init maximum height of the elevator
        // Once true it sets the max angle to be the init max angle and same for the min angle.
        } else if ((elevatorCurrentHeight >= ElevatorConstants.kMinHeightInch && elevatorCurrentHeight <= GameConstants.kLimitUnderDZ)
        || (elevatorCurrentHeight > GameConstants.kLimitAboveDZ && elevatorCurrentHeight <= ElevatorConstants.kMaxHeightInch)){
            AngleMax = ArmConstants.kAngleMax;
            AngleMin = ArmConstants.kAngleMin;
        }
    }
}
