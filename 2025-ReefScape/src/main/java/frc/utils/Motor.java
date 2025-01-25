package frc.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.robot.Constants;

public class Motor {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain/wheel");

    int CANID;
    double positionFactor;
    double velocityFactor; 
    MyMotorType motorType;
    String name;

    public TalonFX motor_talon;
    TalonFXConfiguration config_talon;

    public MyCANSparkMax motor_neo;
    public boolean m_setupMotorDone = false;
    public double angleCalibration;
    public AnalogInput m_analogEncoder;
    SparkClosedLoopController controller_neo;
    SparkBaseConfig config_neo;
    RelativeEncoder encoder_neo;
    int offset_neo;

    DoublePublisher 
    nt_angleinit;

    public Motor (int CANID_, MyMotorType motorType_, String name_) {
        CANID = CANID_;
        motorType = motorType_;
        name = name_;
        positionFactor = 1;
        velocityFactor = 1;
        nt_angleinit = table.getDoubleTopic("angle/init/"+name).publish();

        switch (motorType){       
            case KRAKEN:
                motor_talon = new TalonFX(CANID);
                config_talon = new TalonFXConfiguration();
                motor_talon.getConfigurator().apply(config_talon);
                break;
            case NEO:
                motor_neo = new MyCANSparkMax(CANID, MotorType.kBrushless);
                controller_neo = motor_neo.getClosedLoopController();
                encoder_neo = motor_neo.getEncoder();
                config_neo = new SparkMaxConfig();
                motor_neo.configure(config_neo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

                String text = name_;
                if (text.contains(name+"_driving")) {
                
                Command m_setupMotor = Commands.sequence(
                    Commands.waitUntil(() -> (encoder_neo = motor_neo.getEncoder()) != null),
                    Commands.waitUntil(() -> (motor_neo.configure(config_neo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)) == REVLibError.kOk),
                    new InstantCommand(()-> m_setupMotorDone = true)
                ); 

                } else if (text.contains(name+"_steering")) {
                
                Command m_setupSteering = Commands.sequence(
                    Commands.waitUntil(() -> {offset_neo = m_analogEncoder.getValue(); nt_angleinit.set(offset_neo); return true;}),
                    Commands.waitUntil(() -> (encoder_neo = motor_neo.getEncoder()) != null),
                    Commands.waitUntil(() -> (motor_neo.configure(config_neo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)) == REVLibError.kOk),
                    Commands.waitUntil(() -> encoder_neo.setPosition(-(offset_neo-angleCalibration)/4096.0 * (2*Math.PI)) == REVLibError.kOk),
                    new InstantCommand(()-> m_setupMotorDone = true)
                ); 

                }
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
                if (idleMode == SparkBaseConfig.IdleMode.kCoast) motor_talon.setNeutralMode(NeutralModeValue.Coast);
                else motor_talon.setNeutralMode(NeutralModeValue.Brake); 
                break;
            case NEO:
                config_neo.idleMode(idleMode);       
                break;
        }
        return this;
    } 

    public Motor positionConversionFactor(double _positionFactor){
        switch (motorType) {
            case KRAKEN:
                positionFactor = _positionFactor; // No easy equivalent methods found, Doing the math manually
                break;
            case NEO:
                config_neo.encoder.positionConversionFactor(_positionFactor);       
                break;
        }
        return this;
    } 
    public Motor velocityConversionFactor(double _velocityFactor){
        switch (motorType) {
            case KRAKEN:
                velocityFactor = _velocityFactor; // No easy equivalent methods found, Doing the math manually
                break;
            case NEO:
                config_neo.encoder.positionConversionFactor(_velocityFactor);       
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

    public Motor feedbackSensor(FeedbackSensor sensor){
        switch (motorType) {
            case KRAKEN:  
                // Not really needed As of 1/21/2025
                break;
            case NEO:
                config_neo.closedLoop.feedbackSensor(sensor);     
                break;
        }
        return this;
    } 

    public Motor pidf(double p, double i, double d, double ff){
        switch (motorType) {
            case KRAKEN:
                config_talon.Slot0.withKP(p)
                    .withKI(i)
                    .withKD(d)
                    .withKV(ff);
                break;
            case NEO:
                config_neo.closedLoop.pidf(p,i,d,ff);    
                break;
        }
        return this;
    } 

    public Motor outputRange(double rangeMin, double rangeMax){ // cant find
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.outputRange(rangeMin, rangeMax);     
                break;
        }
        return this;
    } 

    public Motor iZone(double zone){ //
        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                config_neo.closedLoop.iZone(zone);    
                break;
        }
        return this;
    } 

    public Motor positionWrappingEnabled(boolean enabled){ //
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.positionWrappingEnabled(enabled);    
                break;
        }
        return this;
    } 

    public Motor positionWrappingConfig(double min, double max){ //
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
                motor_talon.set(speed);
                break;
            case NEO:
                controller_neo.setReference(speed, SparkMax.ControlType.kVelocity);       
                break;
        }
    }

    public void setPosition(double position){
        switch (motorType) {
            case KRAKEN:
                motor_talon.set(position);
                break;
            case NEO:
                controller_neo.setReference(position, SparkMax.ControlType.kPosition);
                break;
        }
    }

    public double getVelocity(){
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getVelocity().getValueAsDouble()/60.0 * velocityFactor; // Talon method returns rotations Per second
            case NEO:
                return encoder_neo.getVelocity(); // Our math is based on rotations per Minute
        }
        return 0.0;
    }

    public double getPosition(){
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getPosition().getValueAsDouble() * positionFactor;
            case NEO:
                return encoder_neo.getPosition();
        }
        return 0.0;
    }

    public double getOutputCurrent(){
        switch (motorType) {
            case KRAKEN:
                break;
            case NEO:
                return motor_neo.getOutputCurrent();
        }
        return 0.0;
    }
}