package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

interface iDriveMotor {

    void setSpeed(double speed);

    double getVelocity();
    
    double getPosition();
}

class KrakenDriveMotor implements iDriveMotor {
    TalonFX motor;
    
    public void setSpeed(double speed){
        motor.set(speed);
    }

    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }

    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }
}

class NeoDriveMotor implements iDriveMotor {
    SparkMax motor;

    public void setSpeed(double speed){
        motor.set(speed);
    }

    

}
