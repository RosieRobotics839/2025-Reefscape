package frc.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.MotorDefaults;
import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;

public class Motor extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain/wheel");
    static NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/Test");
    Command m_setupMotor;

    int CANID;
    double positionFactor;
    double velocityFactor; 
    MyMotorType motorType;
    String name;

    public static class kFreeRunRPM{
        final double NEO = 5676;
        final double X60 = 6000;
    }

    public TalonFX motor_talon;
    TalonFXConfiguration config_talon;

    public MyCANSparkMax motor_neo;
    private boolean m_setupMotorDone = false;
    public double calibration = 0;
    private boolean m_calibrated = false;
    public AnalogInput m_analogEncoder;
    SparkClosedLoopController controller_neo;
    SparkBaseConfig config_neo;
    RelativeEncoder encoder_neo;
    int m_steeringOffset;
    boolean m_setupScheduled = false;

    double m_testSpeed;
    double m_testPosition;

    public double m_Kp_0, m_Ki_0, m_Kd_0, m_Kff_0;

    DoublePublisher 
    nt_angleinit,
    nt_speedcmd,
    nt_testspeed,
    nt_testposition;
    
    private double m_simPosition, m_simSpeed;
    
    public void scheduleSetup(){
        switch (motorType){
            case KRAKEN:
                m_setupMotor = Commands.sequence( 
                            Commands.waitUntil(() -> motor_talon.getConfigurator().apply(config_talon).isOK()),
                            new InstantCommand(()-> m_setupMotorDone = true)
                            );
                break;
            case NEO:
                m_setupMotor = Commands.sequence(
                    Commands.waitUntil(() -> (encoder_neo = motor_neo.getEncoder()) != null),
                    Commands.waitUntil(() -> {var status = motor_neo.configure(config_neo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); return status == REVLibError.kOk || status == REVLibError.kCannotPersistParametersWhileEnabled;}),
                    Commands.waitUntil(() -> {
                        if(!m_calibrated){
                            nt_angleinit.set(calibration);
                            return encoder_neo.setPosition((calibration)/4096.0 * (2*Math.PI)) == REVLibError.kOk;
                        }
                        return true;
                    }),
                    new InstantCommand(()-> {m_setupMotorDone = true; m_calibrated = true;})
                );
                break;
            default:
                m_setupMotor = Commands.sequence(new InstantCommand(()-> m_setupMotorDone = true));
        }
        m_setupMotorDone = false;
        m_setupMotor.ignoringDisable(true).schedule();
        m_setupScheduled = true;
    }

    public Motor (int CANID_, MyMotorType motorType_, String name_) {
        CANID = CANID_;
        motorType = motorType_;
        name = name_;
        positionFactor = 1;
        velocityFactor = 1;
        nt_angleinit = table.getDoubleTopic("angle/init/"+name).publish();
        nt_speedcmd = table.getDoubleTopic("motor/speedcmd/"+name).publish();

        if (motorType_ != MyMotorType.SIMULATED){
            NTDouble.create(0.0,testtable,"motors/"+name+"/speed",(val)->{m_testSpeed=val; m_testPosition = 0;});
            NTDouble.create(0.0,testtable,"motors/"+name+"/position",(val)->{m_testPosition=val; m_testSpeed = 0;});
            NTBoolean.create(MotorDefaults.inverted,testtable,"motors/"+name+"/inverted",(val)->inverted(val));
            NTDouble.create(MotorDefaults.currentLimit,testtable,"motors/"+name+"/currentLimit",(val)->smartCurrentLimit(val));
            NTDouble.create(MotorDefaults.Kp,testtable,"motors/"+name+"/KP",(val)->withKP(val));
            NTDouble.create(MotorDefaults.Ki,testtable,"motors/"+name+"/KI",(val)->withKI(val));
            NTDouble.create(MotorDefaults.Kd,testtable,"motors/"+name+"/KD",(val)->withKD(val));
            NTDouble.create(MotorDefaults.Kff,testtable, "motors/"+name+"/KFF", (val)->withKFF(val));
        }

        switch (motorType){   
            case KRAKEN:
                motor_talon = new TalonFX(CANID);
                config_talon = new TalonFXConfiguration();

                break;
            case NEO:
                motor_neo = new MyCANSparkMax(CANID, MotorType.kBrushless);
                controller_neo = motor_neo.getClosedLoopController();
                encoder_neo = motor_neo.getEncoder();
                config_neo = new SparkMaxConfig();             
                break;
            default:
        }
        
        this.withKP(0.15)
            .withKFF(0.10);
    }

    public enum MyMotorType {
        KRAKEN, NEO, SIMULATED
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
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
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
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 

    public Motor positionConversionFactor(double _positionFactor){
        positionFactor = _positionFactor;
        switch (motorType) {
            case KRAKEN:
                // No easy equivalent methods found, Doing the math manually
                break;
            case NEO:
                config_neo.encoder.positionConversionFactor(_positionFactor);       
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 
    public Motor velocityConversionFactor(double _velocityFactor){
        velocityFactor = _velocityFactor;
        switch (motorType) {
            case KRAKEN:
                // No easy equivalent methods found, Doing the math manually
                break;
            case NEO:
                config_neo.encoder.velocityConversionFactor(_velocityFactor);       
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 

    public Motor smartCurrentLimit(double stallLimit){
        switch (motorType) {
            case KRAKEN:
                config_talon.CurrentLimits
                    .withStatorCurrentLimit(stallLimit)
                    .withStatorCurrentLimitEnable(true);
                break;
            case NEO:
                config_neo.smartCurrentLimit((int)stallLimit);       
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 

    public Motor setCalibration(double calibrate){
        calibration = calibrate;
        if (m_setupMotorDone) scheduleSetup();
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
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 

    public Motor withKP(double val){
        m_Kp_0 = val;
        switch(motorType){
            case KRAKEN:
                config_talon.Slot0.withKP(m_Kp_0);
                break;
            case NEO:
                config_neo.closedLoop.pidf(m_Kp_0,m_Ki_0,m_Kd_0,m_Kff_0,ClosedLoopSlot.kSlot0);
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }

    public Motor withKI(double val){
        m_Ki_0 = val;
        switch(motorType){
            case KRAKEN:
                config_talon.Slot0.withKI(m_Ki_0);
                break;
            case NEO:
                config_neo.closedLoop.pidf(m_Kp_0,m_Ki_0,m_Kd_0,m_Kff_0,ClosedLoopSlot.kSlot0);
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }
    public Motor withKD(double val){
        m_Kd_0 = val;
        switch(motorType){
            case KRAKEN:
                config_talon.Slot0.withKD(m_Kd_0);
                break;
            case NEO:
                config_neo.closedLoop.pidf(m_Kp_0,m_Ki_0,m_Kd_0,m_Kff_0,ClosedLoopSlot.kSlot0);
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }

    public Motor withKFF(double val){
        m_Kff_0 = val;
        switch(motorType){
            case KRAKEN:
                config_talon.Slot0.withKV(m_Kff_0);
                break;
            case NEO:
                config_neo.closedLoop.pidf(m_Kp_0,m_Ki_0,m_Kd_0,m_Kff_0,ClosedLoopSlot.kSlot0);
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }

    public Motor pidf(double p, double i, double d, double ff){
        m_Kp_0 = p;
        m_Ki_0 = i;
        m_Kd_0 = d;
        m_Kff_0 = ff;
        switch (motorType) {
            case KRAKEN:
                config_talon.Slot0.withKP(m_Kp_0)
                    .withKI(m_Ki_0)
                    .withKD(m_Kd_0)
                    .withKV(m_Kff_0);
                break;
            case NEO:
                config_neo.closedLoop.pidf(m_Kp_0,m_Ki_0,m_Kd_0,m_Kff_0);    
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 

    public Motor outputRange(double rangeMin, double rangeMax){ // cant find
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.outputRange(rangeMin, rangeMax);     
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 

    public Motor iZone(double zone){ //
        switch (motorType) {
            case KRAKEN:
                
                break;
            case NEO:
                config_neo.closedLoop.iZone(zone);    
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    } 

    public Motor positionWrappingEnabled(boolean enabled){ //
        switch (motorType) {
            case KRAKEN:

                break;
            case NEO:
                config_neo.closedLoop.positionWrappingEnabled(enabled);    
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
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
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }     

    public boolean setSpeed(double speed){
        // If motor testing is active, ignore external request.
        if (m_testSpeed != 0 || m_testPosition !=0){
            return true;
        }
        return _setSpeed(speed);
    }

    protected boolean _setSpeed(double speed){
        boolean status;
        switch (motorType) {
            case KRAKEN:
                status = motor_talon.setControl(new VelocityVoltage(speed/(velocityFactor*60))).isOK();
                break;
            case NEO:
                status = controller_neo.setReference(speed, SparkMax.ControlType.kVelocity) == REVLibError.kOk;
                break;
            default:
                m_simSpeed = speed;
                status = true;
        }
        return status;
    }

    public Motor setFollowerMode(int leaderMotorCANID, boolean inverted){
        switch (motorType) {
            case KRAKEN:
                motor_talon.setControl(new Follower(leaderMotorCANID, inverted)).isOK();
                break;
            case NEO:
                config_neo.follow(leaderMotorCANID, inverted); 
                if (m_setupMotorDone) scheduleSetup();
                break;
            default:
        }
        return this;
    }

    public boolean setPosition(double position){
        // If motor testing is active, ignore external request.
        if (m_testSpeed != 0 || m_testPosition !=0){
            return true;
        }
        return _setPosition(position);
    }

    protected boolean _setPosition(double position){
        boolean status;
        switch (motorType) {
            case KRAKEN:
                status = motor_talon.setControl(new PositionVoltage(position/positionFactor)).isOK();
                break;
            case NEO:
                status = controller_neo.setReference(position, SparkMax.ControlType.kPosition) == REVLibError.kOk;
                break;
            default:
                m_simPosition = position;
                m_simSpeed = 0;
                status = true;
        }
        return status;
    }

    public boolean setRelativePosition(double position){
        return setPosition(getPosition() + position);
    }

    public double getVelocity(){
        if (!m_setupMotorDone) return 0;
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getVelocity().getValueAsDouble() * velocityFactor * 60; // Talon method returns rotations Per second
            case NEO:
                return encoder_neo.getVelocity(); // Our math is based on rotations per Minute
            default:
                return m_simSpeed;
        }
    }

    public double getPosition(){
        if (!m_setupMotorDone) return 0;
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getPosition().getValueAsDouble() * positionFactor;
            case NEO:
                return encoder_neo.getPosition();
            default:
                return m_simPosition;
        }
    }

    public double getOutputCurrent(){
        switch (motorType) {
            case KRAKEN:
                break;
            case NEO:
                return motor_neo.getOutputCurrent();
            default:
        }
        return 0.0;
    }

    public boolean isSetupDone() {
        return m_setupMotorDone;
    }
    
    @Override
    public void periodic(){
        if (m_setupScheduled == false && m_setupMotorDone == false) {
            scheduleSetup();
        }
        if (m_setupMotorDone){
            if (m_testSpeed != 0){
                _setSpeed(m_testSpeed);
            } else if (m_testPosition !=0){
                _setPosition(m_testPosition);
            }

            switch (motorType){
                case KRAKEN:
                    break;
                case NEO:
                    break;
                case SIMULATED:
                    if (m_testPosition == 0){
                        m_simPosition = m_simPosition + m_simSpeed * 0.020; // Move simulated motor over a 20ms period.
                    }
                    break;
            }
        }
    }
}