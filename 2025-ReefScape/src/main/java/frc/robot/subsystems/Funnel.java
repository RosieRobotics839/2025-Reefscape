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

    public boolean setTargetPosition(double position){
        m_targetPosition = position;
        return m_motorFunnel.setPosition(position);
    }

    Command FunnelUpCommand = Commands.sequence(
        Commands.waitUntil(() -> {return setTargetPosition(FunnelConstants.kFunnelUp);}),
        Commands.waitUntil(() -> {return atTargetPosition();}));

    Command FunnelDownCommand = Commands.sequence(
        Commands.waitUntil(() -> {return setTargetPosition(FunnelConstants.kFunnelDown);}),
        Commands.waitUntil(() -> {return atTargetPosition();}));

    DoublePublisher
    nt_;

    public Funnel(int CANID) {
    
        m_motorFunnel = new Motor(FunnelConstants.kFunnelCANID, FunnelConstants.kMotorType, "funnel")
            .smartCurrentLimit((int)FunnelConstants.kFunnelMotorCurrentLimit)
            .inverted(true)
            .positionConversionFactor((FunnelConstants.kFunnelGearRatio))
            .pidf(FunnelConstants.kFunnelKp, FunnelConstants.kFunnelKi, FunnelConstants.kFunnelKd, FunnelConstants.kFunnelKff);

    }

    @Override
    public void periodic() {
    
    }

}
