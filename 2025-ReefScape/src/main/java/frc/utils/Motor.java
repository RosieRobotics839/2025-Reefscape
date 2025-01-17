package frc.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.robot.Constants;

public class Motor {
    int CANID;
    MyMotorType motorType;
    String name;

    TalonFX motor_talon;

    MyCANSparkMax motor_neo;
    SparkClosedLoopController controller_neo;
    SparkMaxConfig config_neo;
    RelativeEncoder encoder_neo;

    public Motor (int CANID_, MyMotorType motorType_, String name_) {
        CANID = CANID_;
        motorType = motorType_;
        name = name_;

        switch (motorType_){       
            case KRAKEN:
                
                break;
            case NEO:
                motor_neo = new MyCANSparkMax(CANID, MotorType.kBrushless);
                controller_neo = motor_neo.getClosedLoopController();
                encoder_neo = motor_neo.getEncoder();
                break;
        }
    }

    public enum MyMotorType {
        KRAKEN, NEO
    }

    public Motor inverted(boolean invert){

        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                config_neo.inverted(invert);       
                break;
        }
        return this;
    }

    public Motor idleMode(IdleMode idleMode){

        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                config_neo.idleMode(idleMode);       
                break;
        }
        return this;
    } 

    public Motor positionConversionFactor(double positionFactor){

        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                config_neo.encoder.positionConversionFactor(positionFactor);       
                break;
        }
        return this;
    } 

    public Motor smartCurrentLimit(int stallLimit){

        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                config_neo.smartCurrentLimit(stallLimit);       
                break;
        }
        return this;
    } 

    public void setSpeed(double speed){

        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                controller_neo.setReference(speed, SparkMax.ControlType.kVelocity);       
                break;
        }
    }

    public void setPosition(double position){
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                controller_neo.setReference(position, SparkMax.ControlType.kPosition);
                break;
        }
    }

    public double getVelocity(){
        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                return encoder_neo.getVelocity();
        }
        return 0.0;
    }

    public double getPosition(){
        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                return encoder_neo.getPosition();
        }
        return 0.0;
    }
}