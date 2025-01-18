package frc.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.robot.Constants;

public class Motor {
    int CANID;
    MyMotorType motorType;
    String name;

    TalonFX motor_talon;
    TalonFXConfiguration config_talon;

    MyCANSparkMax motor_neo;
    SparkClosedLoopController controller_neo;
    SparkMaxConfig config_neo;
    RelativeEncoder encoder_neo;

    public Motor (int CANID_, MyMotorType motorType_, String name_) {
        CANID = CANID_;
        motorType = motorType_;
        name = name_;

        switch (motorType){       
            case KRAKEN:
                motor_talon = new TalonFX(CANID);
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
                InvertedValue value;
                if (invert) value = InvertedValue.CounterClockwise_Positive;
                else value = InvertedValue.Clockwise_Positive;
                config_talon.withMotorOutput(config_talon.MotorOutput.withInverted(value));
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

    public Motor smartCurrentLimit(double stallLimit){
        switch (motorType) {
            case KRAKEN:
                config_talon.withCurrentLimits(config_talon.CurrentLimits.withSupplyCurrentLimit(stallLimit));
                break;
            case NEO:
                config_neo.smartCurrentLimit((int)stallLimit);       
                break;
        }
        return this;
    } 

    public Motor feedbackSensor(FeedbackSensor sensor){ // needs params
        switch (motorType) {
            case KRAKEN:  

                break;
            case NEO:
                config_neo.closedLoop.feedbackSensor(sensor); // use params       
                break;
        }
        return this;
    } 

    public Motor pidf(double p, double i, double d, double ff){ // needs params
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.pidf(p,i,d,ff); // use params       
                break;
        }
        return this;
    } 

    public Motor outputRange(double rangeMin, double rangeMax){
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.outputRange(rangeMin, rangeMax);     
                break;
        }
        return this;
    } 

    public Motor iZone(double zone){
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.iZone(zone);    
                break;
        }
        return this;
    } 

    public Motor positionWrappingEnabled(boolean enabled){
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.positionWrappingEnabled(enabled);    
                break;
        }
        return this;
    } 

    public Motor positionWrappingConfig(double min, double max){
        switch (motorType) {
            case KRAKEN:
            
                break;
            case NEO:
                config_neo.closedLoop
                    .positionWrappingMinInput(min)
                    .positionWrappingMaxInput(max);  
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
                return motor_talon.getVelocity().getValueAsDouble()/60.0; // Talon method returns rotations Per second
            case NEO:
                return encoder_neo.getVelocity(); // Our math is based on rotations per Minute
        }
        return 0.0;
    }

    public double getPosition(){
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getPosition().getValueAsDouble();
            case NEO:
                return encoder_neo.getPosition();
        }
        return 0.0;
    }
}