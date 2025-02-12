package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.CalibrationMap;
import frc.utils.Motor;

public class Arm extends SubsystemBase{

    private static Arm instance = new Arm(ArmConstants.kArmCANID, ArmConstants.kAnalogInputID);

    public static Arm getInstance(){
        return instance;
    }
    
    public Motor m_motorArm;
    public AnalogInput m_armAnalogEncoder;
    boolean m_setupArmDone = false;
    double m_armOffset = 0; //change later
    double m_newArmOffset = 0;

    /**
     * This function sets the Arm Angle to be within the target bounds of the minimum and maximum values which are being defined in Constants.
     * @param target Target angle in radians
     */
    public void setArmAngle(double target) {
        if (!m_setupArmDone) return;
        target = Math.max(ArmConstants.kAngleMin, Math.min(ArmConstants.kAngleMax, target));
        m_motorArm.setPosition(target);
    }

    public Arm(int CANID, int analogID) {

        CalibrationMap m_armCalibrationMap = new CalibrationMap(ArmConstants.kArmCalibrationX, ArmConstants.kArmCalibrationY);
        
        m_armAnalogEncoder = new AnalogInput(analogID);
        m_armOffset = m_armAnalogEncoder.getValue();
        if (m_armOffset > m_armCalibrationMap.xmin() && m_armOffset < m_armCalibrationMap.xmax()) {
            m_newArmOffset = m_armOffset;
            }

        m_motorArm = new Motor(ArmConstants.kArmCANID, Motor.MyMotorType.KRAKEN, "arm")
            .smartCurrentLimit((int)ArmConstants.kArmMotorCurrentLimit)
            .inverted(true)
            .positionConversionFactor((ArmConstants.kArmEncoderPositionFactor))
            .velocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor)
            .setCalibration(m_newArmOffset)
            .pidf(ArmConstants.kArmKp, ArmConstants.kArmKi, ArmConstants.kArmKd, ArmConstants.kArmKff);

        } 

        @Override
        public void periodic() {
            
        }
}
