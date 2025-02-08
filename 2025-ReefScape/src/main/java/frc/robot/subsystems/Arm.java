package frc.robot.subsystems;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID_t;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import frc.utils.Motor;

public class Arm extends SubsystemBase{

    private static Arm instance = new Arm(null);

    public static Arm getInstance(){
        return instance;
    }
    
    public Motor m_motorArm;
    public AnalogInput m_analogEncoder;
    boolean m_setupArmDone = false;

    public void setArmAngle(double radians) {
        if (!m_setupArmDone) return;
        radians = Math.max(ArmConstants.kAngleMin, Math.min(ArmConstants.kAngleMax, radians));
        m_angleTarget = radians;
        m_pidArm.setReference(radians, CANSparkMax.ControlType.kPosition);
    }

    public Arm(CANID_t CANID) {

        m_analogEncoder = new AnalogInput(CANID.encoder);
        
        m_motorArm = new Motor(ArmConstants.kArmCANID, Motor.MyMotorType.KRAKEN, "arm");

        m_motorArm
            //.smartCurrentLimit((int)kSwerveModule.kDrivingMotorCurrentLimit)
            .inverted(true)
            .positionConversionFactor((ArmConstants.kArmEncoderPositionFactor))
            .velocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor)
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ArmConstants.kArmKp, ArmConstants.kArmKi, ArmConstants.kArmKd, ArmConstants.kArmKff);
            //.outputRange(-1,1);
            //.iZone(0.15);

            /* Command m_setupArm = Commands.sequence(
                // Motor setup (ensure the motor is restored to factory defaults)
                Commands.waitUntil(() -> m_motorArm.restoreFactoryDefaults() == REVLibError.kOk),
            
                // Encoder setup for position tracking
                Commands.waitUntil(() -> {m_armEncoder = m_motorArm.getEncoder();
                        return m_armEncoder != null;}),
            
                // Calibrate encoders and offset for min/max position
                Commands.waitUntil(() -> {m_armOffset = m_armAnalogEncoder.getValue(); return m_armOffset > m_armCalibrationMap.xmin() && m_armOffset < m_armCalibrationMap.xmax();}),
                Commands.waitUntil(() -> m_armEncoder.setPosition(m_armCalibrationMap.get(m_armOffset)) == REVLibError.kOk),
            
                // Mark the setup as done
                new InstantCommand(() -> m_setupArmDone = true)
                ); */
        
        }

        @Override
        public void periodic() {}
}
