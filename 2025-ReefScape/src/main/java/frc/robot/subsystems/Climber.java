package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.utils.CalibrationMap;
import frc.utils.Motor;

public class Climber extends SubsystemBase{
    
    private static Climber instance = new Climber(ClimberConstants.kClimberCANID, ClimberConstants.kAnalogInputID);

    public static Climber getInstance(){
        return instance;
    }

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Climber/table");

    public Motor m_motorClimber;
    public AnalogInput m_climberAnalogEncoder;
    public int m_climberOffset;
    public int m_newClimberOffset;

    private double m_targetAngle;

    public boolean atTargetAngle(){ 
        return Math.abs(getCurrentAngle()-m_targetAngle) < ClimberConstants.kClimberAngleTolerance;
    }

    public double getCurrentAngle(){
        return m_motorClimber.getPosition(); // TODO: use calibration map for climber angle to spool rotation
    }

    public void setTargetAngle(double position){
        m_targetAngle = position;
        double m_motorTarget = m_targetAngle; // TODO: use calibration map for spool rotations to climber angle
        m_motorClimber.setPosition(m_motorTarget);
    }
        
    public Command ClimberInCommand = Commands.sequence(
        Commands.runOnce(() -> setTargetAngle(ClimberConstants.kClimberAngleIn)),
        Commands.waitUntil(this::atTargetAngle)
    );
        
    public Command ClimberOutCommand = Commands.sequence(
        Commands.runOnce(() -> setTargetAngle(ClimberConstants.kClimberAngleOut)),
        Commands.waitUntil(this::atTargetAngle)
    );

    DoublePublisher
    nt_climberOffset;

    public Climber(int CANID, int analogID) {

        nt_climberOffset = table.getDoubleTopic("angle/climberOffset").publish();

        CalibrationMap m_climberCalibrationMap = new CalibrationMap(ClimberConstants.kClimberCalibrationX, ClimberConstants.kClimberCalibrationY);

        m_climberAnalogEncoder = new AnalogInput(analogID);
        m_climberOffset = m_climberAnalogEncoder.getValue();
        nt_climberOffset.set(m_climberOffset);
        if (m_climberOffset > m_climberCalibrationMap.xmin() && m_climberOffset < m_climberCalibrationMap.xmax()) {
            m_newClimberOffset = m_climberOffset;
        }

        m_motorClimber = new Motor(ClimberConstants.kClimberCANID, ClimberConstants.kMotorType, "climber")
            .withStatorLimit((int)ClimberConstants.kClimberMotorCurrentLimit)
            .inverted(true)
            .idleBrake(true)
            .withGearRatio((ClimberConstants.kClimberGearRatio))
            .withSpeedLimit(ClimberConstants.kMaxSpeed)
            .setCalibration(m_newClimberOffset);

    }

    @Override
    public void periodic() {
    
    }
}
