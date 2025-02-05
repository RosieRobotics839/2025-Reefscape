package frc.robot.subsystems;
import frc.robot.Constants;
import frc.utils.Motor;

public class EndEffector {
    
    public Motor m_motorEffector;

        public EndEffector() {
        
        m_motorEffector = new Motor(Constants.EffectorConstants.kEffectorCANID, Motor.MyMotorType.NEO, "effector");

    }
}
