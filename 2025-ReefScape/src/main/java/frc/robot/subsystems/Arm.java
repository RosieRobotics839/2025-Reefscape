package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.CalibrationMap;
import frc.utils.Motor;
import frc.robot.Constants.GameConstants;

public class Arm extends SubsystemBase{

    private static Arm instance = new Arm(ArmConstants.kArmCANID, ArmConstants.kAnalogInputID);

    public static Arm getInstance(){
        return instance;
    }
    
    public Motor m_motorArm;
    public AnalogInput m_armAnalogEncoder;
    public double m_currentAngle;
    public double m_angleTarget;
    double m_armOffset = 0; //change later
    double m_newArmOffset = 0;
    private boolean m_atScorePosition = false;
    boolean m_setupArmDone = false;
    boolean scoringTrough = false;
    boolean scoringLevels2or3 = false;
    boolean scoringLevel4 = false;
    GameConstants.ScoreLevel m_scoreReefLevel;

    /**
     * This function sets the Arm Angle to be within the target bounds of the minimum and maximum values which are being defined in Constants.
     * @param target Target angle in radians
     */
    public void setArmAngle(double target) {
        if (!m_setupArmDone) return;
        target = Math.max(ArmConstants.kAngleMin, Math.min(ArmConstants.kAngleMax, target));
        m_motorArm.setPosition(target);
        m_angleTarget = target;
    }

    public Boolean atScorePosition(){
      return m_atScorePosition;
    }

    Command ArmPositionCommand = Commands.sequence(
        Commands.waitUntil(() -> {
            switch(m_scoreReefLevel){
                case TROUGH:
                    m_angleTarget = ArmConstants.kTargetAngleTrough;
                    return m_motorArm.setPosition(ArmConstants.kTargetAngleTrough);
                case LEVEL2:
                case LEVEL3:
                    m_angleTarget = ArmConstants.kTargetAngleLevels2or3;
                    return m_motorArm.setPosition(ArmConstants.kTargetAngleLevels2or3);
                case LEVEL4:
                    m_angleTarget = ArmConstants.kTargetAngleLevel4;
                    return m_motorArm.setPosition(ArmConstants.kTargetAngleLevel4);
            }
            return false;
        }),
        Commands.waitUntil(() -> {return atScorePosition();})
    );

    public Arm(int CANID, int analogID) {

        CalibrationMap m_armCalibrationMap = new CalibrationMap(ArmConstants.kArmCalibrationX, ArmConstants.kArmCalibrationY);
        
        m_armAnalogEncoder = new AnalogInput(analogID);
        m_armOffset = m_armAnalogEncoder.getValue();
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
        
        if (m_motorArm.isSetupDone()){
            m_currentAngle = m_motorArm.getPosition();
        }

          if ((Math.abs(m_angleTarget - m_currentAngle)) < ArmConstants.kAngleTolerance){
            m_atScorePosition = true;
          }  

        }
}
