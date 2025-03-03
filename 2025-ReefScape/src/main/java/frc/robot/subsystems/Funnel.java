package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;
import frc.utils.Hysteresis;
import frc.utils.Motor;
import frc.utils.NTValues.NTBoolean;

public class Funnel extends SubsystemBase{
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Funnel/table");

    private static Funnel instance = new Funnel(FunnelConstants.kFunnelCANID);

    public static Funnel getInstance(){
        return instance;
    }

    public Motor m_motorFunnel;
    private double m_targetPosition;

    public Debouncer m_debouncer = new Debouncer(FunnelConstants.kFunnelStallSec, Debouncer.DebounceType.kRising);
    public NTBoolean m_isStalled = new NTBoolean(false, table, "stalled", null);
    public NTBoolean nt_hysteresis = new NTBoolean(false, table, "climberOut", null);

    public boolean atTargetPosition(){ 
        return Math.abs(getPosition()-m_targetPosition) < FunnelConstants.kFunnelAngleTolerance;
    }

    public boolean motorStalled(){
        return m_isStalled.get();
    }

    public double getPosition(){
        return m_motorFunnel.getPosition();
    }

    public void setPosition(double position){
        m_isStalled.set(false);
        m_targetPosition = position;
        m_motorFunnel.setPosition(position);
    }

    public void setRelativePosition(double position){
        m_isStalled.set(false);
        m_targetPosition = m_motorFunnel.getPosition()+position;
        m_motorFunnel.setPosition(m_targetPosition);
    }

    public Command FunnelDownCommand, FunnelUpCommand;
    {
        FunnelUpCommand = Commands.sequence(
            new InstantCommand(() -> setRelativePosition(2.0*FunnelConstants.kFunnelUp)),
            Commands.waitUntil(this::motorStalled).withTimeout(3)
        ).beforeStarting(()->FunnelDownCommand.cancel()).handleInterrupt(()->setPosition(getPosition()));
        
        FunnelDownCommand = Commands.sequence(
            new InstantCommand(() -> setPosition(FunnelConstants.kFunnelDown)),
            Commands.waitUntil(this::atTargetPosition).withTimeout(3)
        ).beforeStarting(()->FunnelUpCommand.cancel()).handleInterrupt(()->setPosition(getPosition()));
    }

    private Hysteresis m_climberHysteresis = new Hysteresis()
    .withThreshold(Units.degreesToRotations(0))
    .withHysteresis(Units.degreesToRotations(5))
    .onFalse(()->FunnelUpCommand.schedule());
    
    public Funnel(int CANID) {
    
        m_motorFunnel = new Motor(FunnelConstants.kFunnelCANID, FunnelConstants.kMotorType, "funnel")
            .withStatorLimit((int)FunnelConstants.kMotorCurrentLimit)
            .inverted(true)
            .withGearRatio(FunnelConstants.kFunnelGearRatio)
            .withSpeedLimit(FunnelConstants.kMaxSpeed)
            .pidf(FunnelConstants.kFunnelPosKp, FunnelConstants.kFunnelPosKi, FunnelConstants.kFunnelPosKd, FunnelConstants.kFunnelPosKff, Motor.GainSlot.POSITION);

    }

    @Override
    public void periodic() {
        boolean currentLimited = m_motorFunnel.getOutputCurrent() > FunnelConstants.kMotorCurrentLimit * FunnelConstants.kStallCurrentRatio;
        boolean zeroSpeed = Math.abs(m_motorFunnel.getVelocity()) <= FunnelConstants.kStallSpeed;
        m_isStalled.set(m_debouncer.calculate(currentLimited && zeroSpeed));

        // Protect funnel based on climber angle input with hysteresis band.
        nt_hysteresis.set(m_climberHysteresis.calculate(Climber.getInstance().motorCal.get(Climber.getInstance().m_angleSensor.get())));

        if (m_isStalled.get()){
            m_motorFunnel.setEncoderPosition(FunnelConstants.kFunnelUp);
            m_motorFunnel.setPosition(FunnelConstants.kFunnelUp);
        }
    }

}
