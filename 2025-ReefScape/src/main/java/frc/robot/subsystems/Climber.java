package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
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
import frc.utils.NTValues.NTDouble;

public class Climber extends SubsystemBase{

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Climber");

    private static Climber instance = new Climber(ClimberConstants.kCANID, ClimberConstants.kDigitalInputID);

    public static Climber getInstance(){
        return instance;
    }

    public Motor m_motor;
    public DutyCycleEncoder m_angleSensor;
    public Boolean m_setupDone = false;

    private double m_targetAngle;

    DoublePublisher nt_positionSensor = table.getDoubleTopic("positionSensor").publish();
    DoublePublisher nt_currentAngle = table.getDoubleTopic("currentAngle").publish();

    NTDouble nt_targetAngle = new NTDouble(0,table,"targetAngle",(val)->setPosition(Units.degreesToRadians(val)));

    public boolean atTargetAngle(){ 
        return Math.abs(getPosition()-m_targetAngle) < ClimberConstants.kAngleTolerance;
    }

    public double getPosition(){
        return m_motor.getPosition()*2.0*Math.PI; // TODO: use calibration map for climber angle to spool rotation
    }

    /**
     * Sets the target position of the climber, protected by kAngleMax kAngleMin limits
     * @param position in radians
     */
    public void setPosition(double radians){
        m_targetAngle = Math.max(ClimberConstants.kAngleMin,Math.min(ClimberConstants.kAngleMax,radians));
        double m_motorTarget = m_targetAngle/(2*Math.PI);
        m_motor.setPosition(m_motorTarget);
    }
        
    public Command ClimberInCommand = Commands.sequence(
        new InstantCommand(() -> setPosition(ClimberConstants.kAngleMin))
    );
        
    public Command ClimberOutCommand = Commands.sequence(
        new InstantCommand(() -> setPosition(ClimberConstants.kAngleMax))
    );

    public Calibrate motorCal;
    public Climber(int CANID, int analogID) {

        m_angleSensor = new DutyCycleEncoder(analogID);

        m_motor = new Motor(ClimberConstants.kCANID, ClimberConstants.kMotorType, "climber")
            .withStatorLimit((int)ClimberConstants.kMotorCurrentLimit)
            .inverted(true)
            .idleBrake(true)
            .withGearRatio((ClimberConstants.kGearRatio))
            .withAlternateEncoder(ClimberConstants.kRelativeEncoderCPR,true)
            .positionWrappingEnabled(true)
            .withSpeedLimit(ClimberConstants.kMaxSpeed);

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
        nt_currentAngle.set(Units.radiansToDegrees(getPosition()));
        nt_targetAngle.set(Units.radiansToDegrees(getPosition()));
    }
}
