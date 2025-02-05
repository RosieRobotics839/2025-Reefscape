package frc.robot.subsystems;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANID_t;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import frc.utils.Motor;
import frc.utils.Motor.MyMotorType;

public class Arm extends SubsystemBase{
    
    public Motor m_motorArm;
    public AnalogInput m_analogEncoder;

    public Arm(CANID_t CANID) {

        m_analogEncoder = new AnalogInput(CANID.encoder);
        
        m_motorArm = new Motor(ArmConstants.kArmCANID, Motor.MyMotorType.KRAKEN, "arm");

        m_motorArm
            //.smartCurrentLimit((int)kSwerveModule.kDrivingMotorCurrentLimit)
            .positionConversionFactor((ArmConstants.kArmEncoderPositionFactor))
            .velocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor)
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ArmConstants.kArmKp, ArmConstants.kArmKi, ArmConstants.kArmKd, ArmConstants.kArmKff);
            //.outputRange(-1,1)
            //.iZone(0.15); 

    }

    @Override
    public void periodic() {

    }
}
