package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public boolean scoringTrough(){
        return false; //add when bboard is ready so we can determine status based off of stage dial.
    }

    public boolean scoringLevels2or3(){
        return false; //add when bboard is ready so we can determine status based off of stage dial.
    }

    public boolean scoringLevel4(){
        return false; //add when bboard is ready so we can determine status based off of stage dial.
    }

    Command ArmPositionCommand = Commands.sequence(
        Commands.waitUntil(() -> {
            if (scoringTrough()){
                m_motorArm.setPosition(ArmConstants.kArmAngleTrough);
                return true;
            } else if (scoringLevels2or3()){
                m_motorArm.setPosition(ArmConstants.kArmAngleLevel2or3);
                return true;
            } else if (scoringLevel4()){
                m_motorArm.setPosition(ArmConstants.kArmAngleLevel4);
                return true;
            }
            return false;
        }));

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
            
        }
}
