package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.utils.Calibrate;
import frc.utils.Motor;

public class Climber extends SubsystemBase{

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Climber/table");

    private static Climber instance = new Climber(ClimberConstants.kCANID, ClimberConstants.kDigitalInputID);

    public static Climber getInstance(){
        return instance;
    }

    public Motor m_motor;
    public DutyCycleEncoder m_angleSensor;
    public Boolean m_setupDone = false;

    private double m_targetAngle;

    DoublePublisher
    nt_positionSensor;

    public boolean atTargetAngle(){ 
        return Math.abs(getCurrentAngle()-m_targetAngle) < ClimberConstants.kAngleTolerance;
    }

    public double getCurrentAngle(){
        return m_motor.getPosition()*2.0*Math.PI; // TODO: use calibration map for climber angle to spool rotation
    }

    public void setTargetAngle(double position){
        m_targetAngle = position;
        m_motor.setPosition(m_targetAngle/(2.0*Math.PI));
    }
        
    public Command ClimberInCommand = Commands.sequence(
        Commands.runOnce(() -> setTargetAngle(ClimberConstants.kAngleIn)),
        Commands.waitUntil(this::atTargetAngle)
    );
        
    public Command ClimberOutCommand = Commands.sequence(
        Commands.runOnce(() -> setTargetAngle(ClimberConstants.kAngleOut)),
        Commands.waitUntil(this::atTargetAngle)
    );

    public Calibrate motorCal;
    public Climber(int CANID, int analogID) {

        nt_positionSensor = table.getDoubleTopic("angle/positionSensor").publish();

        m_angleSensor = new DutyCycleEncoder(analogID);

        m_motor = new Motor(ClimberConstants.kCANID, ClimberConstants.kMotorType, "climber")
            .withStatorLimit((int)ClimberConstants.kMotorCurrentLimit)
            .inverted(true)
            .idleBrake(true)
            .withGearRatio((ClimberConstants.kGearRatio))
            .withSpeedLimit(ClimberConstants.kMaxSpeed)
            .positionWrappingEnabled(true);

        motorCal = Calibrate.motor("climber",
            ClimberConstants.kCalibrationX,
            ClimberConstants.kCalibrationY,
            m_motor,
            ()->m_angleSensor.get(),
            new InstantCommand(()->m_setupDone = true)
        );
    }

    @Override
    public void periodic() {
        nt_positionSensor.set(m_angleSensor.get());
    }
}
