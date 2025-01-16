package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;

interface iSteerMotor {

    double getPosition();

    void setState(SwerveModuleState targetState);
}

class KrakenSteerMotor implements iSteerMotor {
    
}

class NeoSteerMotor implements iSteerMotor {
    
}