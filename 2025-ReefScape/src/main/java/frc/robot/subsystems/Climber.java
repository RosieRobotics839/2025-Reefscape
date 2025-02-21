package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.utils.CalibrationMap;
import frc.utils.Motor;

public class Climber extends SubsystemBase{

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Climber/table");

    private static Climber instance = new Climber(ClimberConstants.kClimberCANID, ClimberConstants.kDigitalInputID);

    public static Climber getInstance(){
        return instance;
    }

    public Motor m_motorClimber;
    public DutyCycleEncoder m_climberAnalogEncoder;
    public double m_climberOffset;
    public double m_newClimberOffset;

    private double m_targetAngle;

    CalibrationMap m_climberCalibrationMap = new CalibrationMap(ClimberConstants.kClimberCalibrationX, ClimberConstants.kClimberCalibrationY);


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

    public boolean calibrationValid(){
        return m_climberAnalogEncoder.get() > m_climberCalibrationMap.xmin() && m_climberAnalogEncoder.get() < m_climberCalibrationMap.xmax();
    }

    DoublePublisher
    nt_climberOffset;

    public Command CalibrateClimber = Commands.sequence(
        Commands.waitUntil(this::calibrationValid),
        Commands.waitUntil(()->m_motorClimber.isSetupDone()),
        Commands.waitUntil(()->m_motorClimber.setEncoderPosition(m_climberCalibrationMap.get(m_climberAnalogEncoder.get())))
    );

    public Climber(int CANID, int analogID) {

        nt_climberOffset = table.getDoubleTopic("angle/climberOffset").publish();

        m_climberAnalogEncoder = new DutyCycleEncoder(analogID);

        m_motorClimber = new Motor(ClimberConstants.kClimberCANID, ClimberConstants.kMotorType, "climber")
            .withStatorLimit((int)ClimberConstants.kClimberMotorCurrentLimit)
            .inverted(true)
            .idleBrake(true)
            .withGearRatio((ClimberConstants.kClimberGearRatio))
            .withSpeedLimit(ClimberConstants.kMaxSpeed)
            .setCalibration(m_newClimberOffset*4096.0);

        CalibrateClimber.schedule();
    }

    @Override
    public void periodic() {
        nt_climberOffset.set(m_climberAnalogEncoder.get());
    }
}
