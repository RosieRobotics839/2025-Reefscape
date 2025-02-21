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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.MotorDefaults;
import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;
import frc.utils.NTValues.NTInteger;

public class Motor extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain/wheel");
    static NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/Test");
    Command m_setupMotor;

    int CANID;
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
    boolean m_enabSlowSpeed;
    boolean m_setupScheduled = false;

    double m_maxPositiveSpeed = Double.POSITIVE_INFINITY;
    double m_maxNegativeSpeed = Double.NEGATIVE_INFINITY;
    SlewRateLimiter m_positionlimiter = new SlewRateLimiter(m_maxPositiveSpeed, m_maxNegativeSpeed, 0);

    double m_gearReduction;
    double m_speedTarget;
    double m_positionTarget;
    boolean m_lowspeedreverse;
    ControlType m_controlType = ControlType.NONE;
    GainSlot m_gainslot = GainSlot.POSITION;

    double m_testSpeed;
    double m_testPosition;
    boolean m_testEnable;
    boolean m_appliedPositionGains = false;
    boolean m_appliedVelocityGains = false;
    boolean m_inConstructor;

    public static class Gains {
        public double Kp, Ki, Kd, Kff;
        public Gains(){this.Kp = this.Ki = this.Kd = this.Kff = 0;}
        public Gains(double Kp, double Ki, double Kd, double Kff){
            this.Kp = Kp; this.Ki = Ki; this.Kd = Kd; this.Kff = Kff;
        }
        public Gains(Gains gains){this.Kp=gains.Kp; this.Ki=gains.Ki; this.Kd=gains.Kd; this.Kff=gains.Kff;}
        public Gains scale(double s){return new Gains(Kp*s, Ki*s, Kd*s, Kff*s);}
    }

    Gains [] m_gains = new Gains[4];

    DoublePublisher
    nt_angleinit,
    nt_speedcmd;

    NTInteger
    nt_slot,
    nt_controltype;

    NTBoolean
    nt_testEnabled,
    nt_enabslowspeed,
    nt_inverted,
    nt_idleBrake;

    NTDouble
    nt_testSpeed,
    nt_testPosition,
    nt_position,
    nt_speed,
    nt_current,
    nt_curLimit,
    nt_izone,
    nt_speedLim,
    nt_speedLimPos,
    nt_speedLimNeg,
    nt_Kp,
    nt_Ki,
    nt_Kd,
    nt_Kff,
    nt_outputRange;

    private double m_simPosition, m_simSpeed;

    public void scheduleSetup(){
        if (!m_appliedVelocityGains && !m_appliedPositionGains){
            // Set default motor gains
            for (int i=3; i>=0; i--){
                this.pidf(m_gains[i].scale(m_gearReduction), GainSlot.values()[i]);
            }
        } else
        if (m_appliedVelocityGains && !m_appliedPositionGains){
            // If only velocity gains were applied
            pidf(new Gains(m_gains[GainSlot.SPEED.ordinal()].Kp * MotorDefaults.NEO.kPositionGainRatio, 0, 0, 0), GainSlot.POSITION);
        }

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
                            return encoder_neo.setPosition((calibration)/4096.0) == REVLibError.kOk;
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
        m_inConstructor = true;
        CANID = CANID_;
        motorType = motorType_;
        name = name_;
        nt_angleinit = table.getDoubleTopic("angle/init/"+name).publish();
        nt_speedcmd = table.getDoubleTopic("motor/speedcmd/"+name).publish();

        nt_enabslowspeed = new NTBoolean(m_enabSlowSpeed,testtable,"motors/"+name+"/enabslowspeed",(val)->m_enabSlowSpeed=val);
        nt_testEnabled  = new NTBoolean(false,testtable,"motors/"+name+"/testenable",(val)->m_testEnable = val);
        nt_testSpeed    = new NTDouble(0.0,testtable,"motors/"+name+"/testspeed",(val)->{if (m_testEnable)
            _setSpeed(val, GainSlot.SPEED);
             else nt_testSpeed.set(0);});
        nt_testPosition = new NTDouble(0.0,testtable,"motors/"+name+"/testpos",(val)->{if (m_testEnable) _setPosition(val, GainSlot.POSITION); else nt_testPosition.set(0);});
        nt_position     = new NTDouble(0.0,testtable,"motors/"+name+"/position",(val)->{var t = getPosition(); if (m_testEnable) setEncoderPosition(val); else nt_position.set(t);});
        nt_speed        = new NTDouble(0.0,testtable,"motors/"+name+"/speed",(val)->{});
        nt_izone        = new NTDouble(1.0,testtable,"motors/"+name+"/izone",(val)->withIZone(val));
        nt_speedLim     = new NTDouble(0.0,testtable,"motors/"+name+"/speedlim/maxboth",(val)->{if (val >= 0) withSpeedLimit(val,-val);});
        nt_speedLimPos  = new NTDouble(0.0,testtable,"motors/"+name+"/speedlim/maxpos",(val)->{if (val >= 0) withSpeedLimit(val,m_maxNegativeSpeed); else nt_speedLimPos.set(m_maxPositiveSpeed);});
        nt_speedLimNeg  = new NTDouble(0.0,testtable,"motors/"+name+"/speedlim/maxneg",(val)->{if (val <= 0) withSpeedLimit(m_maxPositiveSpeed,val); else nt_speedLimNeg.set(m_maxNegativeSpeed);});
        nt_inverted     = new NTBoolean(MotorDefaults.kInverted,testtable,"motors/"+name+"/inverted",(val)->inverted(val));
        nt_idleBrake    = new NTBoolean(MotorDefaults.kIdleBrake, testtable,"motors/"+name+"/idlebrake",(val)->idleBrake(val));
        nt_outputRange  = new NTDouble(MotorDefaults.kOutputRange, testtable,"motors/"+name+"/outrange",(val)->withOutputRange(-val,val));
        nt_curLimit     = new NTDouble(MotorDefaults.kCurrentLimit,testtable,"motors/"+name+"/currentLimit",(val)->withStatorLimit(val));
        nt_current      = new NTDouble(0.0,testtable,"motors/"+name+"/current",(val)->{});
        nt_slot         = new NTInteger(0, testtable,"motors/"+name+"/gains/slot",(val)->{nt_Kp.set(m_gains[val].Kp);nt_Ki.set(m_gains[val].Ki);nt_Kd.set(m_gains[val].Kd);nt_Kff.set(m_gains[val].Kff);});
        nt_controltype  = new NTInteger(-1, testtable,"motors/"+name+"/controltype",(val)->{});
        nt_Kp           = new NTDouble(0.0,testtable,"motors/"+name+"/gains/KP",(val)->withKP(val,GainSlot.values()[nt_slot.get()]));
        nt_Ki           = new NTDouble(0.0,testtable,"motors/"+name+"/gains/KI",(val)->withKI(val,GainSlot.values()[nt_slot.get()]));
        nt_Kd           = new NTDouble(0.0,testtable,"motors/"+name+"/gains/KD",(val)->withKD(val,GainSlot.values()[nt_slot.get()]));
        nt_Kff          = new NTDouble(0.0,testtable, "motors/"+name+"/gains/KFF", (val)->withKFF(val,GainSlot.values()[nt_slot.get()]));

        switch (motorType){
            case KRAKEN:
                motor_talon = new TalonFX(CANID);
                config_talon = new TalonFXConfiguration();
                m_gains[0] = new Gains(MotorDefaults.Kraken.kGainPosition);
                m_gains[1] = new Gains(MotorDefaults.Kraken.kGainSpeed);
                m_gains[2] = new Gains(MotorDefaults.Kraken.kGainAux1);
                m_gains[3] = new Gains(MotorDefaults.Kraken.kGainAux2);
                break;
            case NEO:
                motor_neo = new MyCANSparkMax(CANID, MotorType.kBrushless);
                controller_neo = motor_neo.getClosedLoopController();
                encoder_neo = motor_neo.getEncoder();
                config_neo = new SparkMaxConfig();
                m_gains[0] = new Gains(MotorDefaults.NEO.kGainPosition);
                m_gains[1] = new Gains(MotorDefaults.NEO.kGainSpeed);
                m_gains[2] = new Gains(MotorDefaults.NEO.kGainAux1);
                m_gains[3] = new Gains(MotorDefaults.NEO.kGainAux2);
                break;
            default:
                m_gains[0] = new Gains(MotorDefaults.NEO.kGainPosition);
                m_gains[1] = new Gains(MotorDefaults.NEO.kGainSpeed);
                m_gains[2] = new Gains(MotorDefaults.NEO.kGainAux1);
                m_gains[3] = new Gains(MotorDefaults.NEO.kGainAux2);
        }

        withGearRatio(1.0);
        withIZone(MotorDefaults.iZone);
        withSpeedLimit(Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);

        m_inConstructor = false;
    }

    public enum GainSlot {
        POSITION, SPEED, AUX1, NEO4;
    }

    public enum MyMotorType {
        KRAKEN, NEO, SIMULATED
    }

    public enum ControlType {
        NONE, POSITION, SPEED, SLOWSPEED;
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
        nt_inverted.set(invert);
        return this;
    }

    /**
     * Sets the motor idle behavior for robot disabled (Brake or Free Spin)
     * @param brake True to set idle mode to brake
     * @return Motor instance for chaining methods
     */
    public Motor idleBrake(boolean brake){
        switch (motorType) {
            case KRAKEN:
                motor_talon.setNeutralMode(( brake ? NeutralModeValue.Brake : NeutralModeValue.Coast ));
                break;
            case NEO:
                config_neo.idleMode(( brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast));
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        nt_idleBrake.set(brake);
        return this;
    }

    public Motor withSlowSpeedControl(boolean val){
        nt_enabslowspeed.set(val);
        m_enabSlowSpeed = val;
        return this;
    }

    /**
     * This function sets the gear reduction of the motor in rotations to the mechanism it is attached to in rotations. You should not use this for unit conversions to in/s.
     * @param positionFactor A value greater than one is a reduction in gearing (typical)
     * @return Motor instance for chaining methods
     */
    public Motor withGearRatio(double gearReduction){
        m_gearReduction = gearReduction;
        switch (motorType) {
            case KRAKEN:
                config_talon.Feedback.withSensorToMechanismRatio(gearReduction);
                break;
            case NEO:
                config_neo.encoder.positionConversionFactor(1/gearReduction);
                config_neo.encoder.velocityConversionFactor(1/gearReduction/60.0);
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }

    public Motor withStatorLimit(double stallLimit){
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
        nt_curLimit.set(stallLimit);
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

    public Motor withKP(double val, GainSlot slot){
        m_gains[slot.ordinal()].Kp = val;
        return pidf(m_gains[slot.ordinal()], slot);
    }

    public Motor withKI(double val, GainSlot slot){
        m_gains[slot.ordinal()].Ki = val;
        return pidf(m_gains[slot.ordinal()], slot);
    }

    public Motor withKD(double val, GainSlot slot){
        m_gains[slot.ordinal()].Kd = val;
        return pidf(m_gains[slot.ordinal()], slot);
    }

    public Motor withKFF(double val, GainSlot slot){
        m_gains[slot.ordinal()].Kff = val;
        return pidf(m_gains[slot.ordinal()], slot);
    }

    public Motor pidf(Gains gains, GainSlot slot){
        return pidf(gains.Kp,gains.Ki,gains.Kd,gains.Kff,slot);
    }

    public Motor pidf(double p, double i, double d, double ff, GainSlot slot){

        if (slot == GainSlot.POSITION){
            m_appliedPositionGains = true;
        }
        if (slot == GainSlot.SPEED){
            m_appliedVelocityGains = true;
        }

        m_gains[slot.ordinal()].Kp = p;
        m_gains[slot.ordinal()].Ki = i;
        m_gains[slot.ordinal()].Kd = d;
        m_gains[slot.ordinal()].Kff = ff;

        switch (motorType) {
            case KRAKEN:
                switch (slot){
                    case POSITION:
                        config_talon.Slot0.withKP(p).withKI(i).withKD(d).withKV(ff); break;
                    case SPEED:
                        config_talon.Slot1.withKP(p).withKI(i).withKD(d).withKV(ff); break;
                    case AUX1:
                        config_talon.Slot2.withKP(p).withKI(i).withKD(d).withKV(ff); break;
                    default:
                }
                break;
            case NEO:
                config_neo.closedLoop.pidf(p,i,d,ff, ClosedLoopSlot.values()[slot.ordinal()]);
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        nt_Kp.set(p);
        nt_Ki.set(i);
        nt_Kd.set(d);
        nt_Kff.set(ff);
        nt_slot.set(slot.ordinal());
        return this;
    }

    public Motor withOutputRange(double min, double max){
        withOutputRange(min, max, GainSlot.POSITION);
        withOutputRange(min, max, GainSlot.SPEED);
        return this;
    }

    /**
     * Sets a speed limit for the motor with symmetric positive and negative values
     * @param speed "Mechanism rotations per second limit"
     * @return Motor instance for chaining methods
     */
    public Motor withSpeedLimit(double speed){
        withSpeedLimit(speed, -speed); 
        nt_speedLim.set(speed); 
        return this;
    }

    public Motor withSpeedLimit(double positive, double negative){
        m_maxPositiveSpeed = positive;
        m_maxNegativeSpeed = negative;
        nt_speedLimNeg.set(negative);
        nt_speedLimPos.set(positive);

        // Create position limiter with the allowed travel distance at speed over 20 milliseconds.
        m_positionlimiter = new SlewRateLimiter(positive, negative, getPosition());
        return this;
    }

    public Motor withOutputRange(double min, double max, GainSlot slot){ // cant find
        nt_outputRange.set(max);
        switch (motorType) {
            case KRAKEN:
                // TODO: Do we need something here?
                break;
            case NEO:
                config_neo.closedLoop.outputRange(min, max, ClosedLoopSlot.values()[slot.ordinal()]);
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }

    public Motor withIZone(double zone){
        nt_izone.set(zone);
        switch (motorType) {
            case KRAKEN:
                /*  Phoenix 6 "automatically" prevents integral windup in closed-loop controls. As a result, the
                    Integral Zone and Max Integral Accumulator configs are no longer necessary and have been
                    removed. (https://v6.docs.ctr-electronics.com/_/downloads/en/stable/pdf/ Section 5.6.8)
                    It doesn't work very well though, an integrator max limit would be nice, but they removed it...
                */
                break;
            case NEO:
                config_neo.closedLoop.iZone(zone);
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }

    /**
     * A continuous mechanism is a mechanism with unlimited travel in any direction, and whose
     * rotational position can be represented with multiple unique position values. Some examples
     * of continuous mechanisms are swerve drive steer mechanisms or turrets (without cable management).
     * <p>
     * This uses the mechanism rotation value assuming that the mechanism is continuous within 1 rotation.
     * If there is a gear ratio between the sensor and the mechanism, make sure to apply a setGearRatio so
     * the closed loop operates on the full rotation.
     * @param enabled If true, uses a mode of closed loop operation that enables the motor
     * to take the “shortest path”
     * @return Motor instance for chaining methods
     */
    public Motor positionWrappingEnabled(boolean enabled){
        switch (motorType) {
            case KRAKEN:
                config_talon.ClosedLoopGeneral.withContinuousWrap(enabled);
                break;
            case NEO:
                config_neo.closedLoop.positionWrappingInputRange(-0.5, 0.5);
                config_neo.closedLoop.positionWrappingEnabled(enabled);
                break;
            default:
        }
        if (m_setupMotorDone) scheduleSetup();
        return this;
    }

    public void stopMotor(){
        m_speedTarget = 0;
        switch (motorType) {
            case KRAKEN:
                motor_talon.stopMotor();
                break;
            case NEO:
                motor_neo.stopMotor();
                break;
            default:
                m_simSpeed = 0;
        }
    }

    public boolean setSpeed(double speed){return setSpeed(speed, GainSlot.SPEED);} // Default GainSlot
    public boolean setSpeed(double speed, GainSlot slot){
        // If motor testing is active, ignore external request.
        if (m_testEnable){
            return true;
        }
        return _setSpeed(speed, slot);
    }

    protected boolean _setSpeed(double speed, GainSlot slot){

        // Limit speed to our speed limits;
        speed = MathUtil.clamp(speed,m_maxNegativeSpeed,m_maxPositiveSpeed);

        // Check if we should be in SLOWSPEED control mode which uses position control for slow speeds.
        if (m_enabSlowSpeed && Math.abs(speed*m_gearReduction) <= MotorDefaults.kSlowThreshold){
            // Compare to last speed target to see if we've reversed direction
            if (Math.signum(m_speedTarget) != Math.signum(speed)){
                m_lowspeedreverse = true;
            }
            m_speedTarget = speed;
            if (m_controlType != ControlType.SLOWSPEED){
                m_controlType = ControlType.SLOWSPEED;
                //m_positionTarget = getPosition() + getVelocity() * 0.020;
            }
            // If spinning too fast, set speed control mode to reduce speed near SLOWSPEED range first
            if (Math.abs(getVelocity()*m_gearReduction) > MotorDefaults.kSlowThreshold*1.2){
                _setTargetSpeed(speed, slot);
            }
            return true;
        }

        // Normal speed control mode
        m_controlType = ControlType.SPEED;
        return _setTargetSpeed(speed, slot);
    };

    protected boolean _setTargetSpeed(double speed, GainSlot slot){
        boolean status;

        m_gainslot = slot;
        m_speedTarget = speed;
        switch (motorType) {
            case KRAKEN:
                status = motor_talon.setControl(new VelocityVoltage(speed).withSlot(slot.ordinal())).isOK();
                break;
            case NEO:
                status = controller_neo.setReference(speed, SparkMax.ControlType.kVelocity, ClosedLoopSlot.values()[slot.ordinal()]) == REVLibError.kOk;
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

    public void setPosition(double position){
        setPosition(position, GainSlot.POSITION);
    }
    public void setPosition(double position, GainSlot slot){
        // If motor testing is active, ignore external request.
        if (m_testEnable == true){
            return;
        }
        _setPosition(position, slot);
    }

    public void setRelativePosition(double position){
        setPosition(getPosition()+position);
    }

    /**
     * This method takes in a position request and a GainSlot to use, it limits the position request to be within the speed limited allowed range
     * @param position
     * @param slot
     * @return
     */
    protected void _setPosition(double position, GainSlot slot){
        if (m_controlType != ControlType.POSITION){
            // Transitioning into position control, reset our position control speed limiter.
            m_positionlimiter.reset(getPosition());
        }
        m_controlType = ControlType.POSITION;
        m_speedTarget = 0;
        m_positionTarget = position;
        m_gainslot = slot;
    }

    /**
     * This is the method that sends the actual request to the motor controller. It's called by the SLOWSPEED control in periodic as well as the protected method _setPosition().
     * <p>
     * There is no request limiting in this function and should be only called where you need direct motor commands to be send.
     * @param position
     * @param slot
     * @return
     */
    protected boolean _setTargetPosition(double position, GainSlot slot){
        boolean status;
        switch (motorType) {
            case KRAKEN:
                status = motor_talon.setControl(new PositionVoltage(position).withSlot(slot.ordinal())).isOK();
                break;
            case NEO:
                status = controller_neo.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.values()[slot.ordinal()]) == REVLibError.kOk;
                break;
            default:
                m_simPosition = position;
                m_simSpeed = 0;
                status = true;
        }
        return status;
    }

    public boolean setEncoderPosition(double position){
        switch (motorType) {
            case KRAKEN:
                return motor_talon.setPosition(position).isOK();
            case NEO:
                return encoder_neo.setPosition(position) == REVLibError.kOk; // Our math is based on rotations per Minute
            default:
        }
        return false;
    }

    /**
     * Velocity of the mechanism the motor is attached to (i.e. motor speed * gear ratio) in rotations per second.
     * @return Speed in rotations per second scaled by the gear ratio of the mechanism.
     */
    public double getVelocity(){
        if (!m_setupMotorDone) return 0;
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getVelocity().getValueAsDouble(); // Talon method returns rotations Per second
            case NEO:
                return encoder_neo.getVelocity(); // Our math is based on rotations per Minute
            default:
                return m_simSpeed;
        }
    }

    /**
     * Position of the mechanism the motor is attached to (i.e. motor position * gear ratio) in rotations.
     * @return Position in rotations scaled by the gear ratio of the mechanism.
     */
    public double getPosition(){
        if (!m_setupMotorDone) return 0;
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getPosition().getValueAsDouble();
            case NEO:
                return encoder_neo.getPosition();
            default:
                return m_simPosition;
        }
    }

    public double getOutputCurrent(){
        switch (motorType) {
            case KRAKEN:
                return motor_talon.getStatorCurrent().getValueAsDouble();
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
        if (m_setupMotorDone && !m_testEnable){
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

        if (DriverStation.isDisabled()){
            m_controlType = ControlType.NONE;
            m_speedTarget = 0;
            m_positionTarget = getPosition();
            m_positionlimiter.reset(getPosition());
            stopMotor();
        } else {
            // Are we in SLOWSPEED mode, where we use should use position control instead of velocity?
            if (m_controlType == ControlType.SLOWSPEED){
                // Let's first check if the position target makes sense compared to the current position (to avoid some crazy high speed jump in position).
                double m_posdiff = m_positionTarget-getPosition();
                if (Math.abs(m_posdiff) > 1/m_gearReduction){
                    // If the target position is more than 1 motor (not mechanism) rotation from the target we need to transition
                    // from velocity to position control, so update the position target to just ahead of the direction we are turning.
                    m_positionTarget = getPosition() + Math.signum(m_speedTarget)*MotorDefaults.kSlowTransitionExtraSpin/m_gearReduction;
                }
                if (m_lowspeedreverse){
                    // If we're reversing direction, make a small jump across the hysteresis band of the motor to avoid stopping for a moment.
                    m_lowspeedreverse = false;
                    m_positionTarget += Math.signum(m_speedTarget)*MotorDefaults.kSlowHysteresis/m_gearReduction;
                }
                m_positionTarget += m_speedTarget * 0.020;
                _setTargetPosition(m_positionTarget, GainSlot.POSITION);
            }
            else if (m_controlType == ControlType.POSITION){
                double posRequest = m_positionlimiter.calculate(m_positionTarget);
                _setTargetPosition(posRequest, m_gainslot);
            }
        }
        nt_controltype.set(m_controlType.ordinal());
        nt_position.set(getPosition());
        nt_speed.set(getVelocity());
        nt_current.set(getOutputCurrent());
    }
}
