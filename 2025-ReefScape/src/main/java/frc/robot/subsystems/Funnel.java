package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;
import frc.utils.Motor;

public class Funnel extends SubsystemBase{

    private static Funnel instance = new Funnel(FunnelConstants.kFunnelCANID);

    public static Funnel getInstance(){
        return instance;
    }

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Funnel/table");

    public Motor m_motorFunnel;
    private double m_targetPosition;

    public boolean atTargetPosition(){ 
        return Math.abs(getCurrentPosition()-m_targetPosition) < FunnelConstants.kFunnelAngleTolerance;
    }

    public double getCurrentPosition(){
        return m_motorFunnel.getPosition();
    }

    public void setTargetPosition(double position){
        m_targetPosition = position;
        m_motorFunnel.setPosition(position);
    }

    public Command FunnelUpCommand = Commands.sequence(
        Commands.runOnce(() -> setTargetPosition(FunnelConstants.kFunnelUp)),
        Commands.waitUntil(this::atTargetPosition)
    );
        
    public Command FunnelDownCommand = Commands.sequence(
        Commands.runOnce(() -> setTargetPosition(FunnelConstants.kFunnelDown)),
        Commands.waitUntil(this::atTargetPosition)
    );

    DoublePublisher
    nt_;

    public Funnel(int CANID) {
    
        m_motorFunnel = new Motor(FunnelConstants.kFunnelCANID, FunnelConstants.kMotorType, "funnel")
            .withStatorLimit((int)FunnelConstants.kFunnelMotorCurrentLimit)
            .inverted(true)
            .withGearRatio(FunnelConstants.kFunnelGearRatio)
            .withSpeedLimit(FunnelConstants.kMaxSpeed)
            .pidf(FunnelConstants.kFunnelPosKp, FunnelConstants.kFunnelPosKi, FunnelConstants.kFunnelPosKd, FunnelConstants.kFunnelPosKff, Motor.GainSlot.POSITION);

    }

    @Override
    public void periodic() {
    
    }

}
