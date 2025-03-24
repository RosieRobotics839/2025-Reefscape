package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.utils.CalibrationMap;
import frc.utils.Motor;
import frc.utils.Motor.GainSlot;
import frc.utils.NTValues.NTDouble;

public class Climber extends SubsystemBase{

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Climber");

    private static Climber instance = new Climber(ClimberConstants.kCANID, ClimberConstants.kDigitalInputID);

    public static Climber getInstance(){
        return instance;
    }

    public Motor m_motor;
    public boolean hasReachedInPos = false;
    public boolean hasReachedOutPos = false;
    private double lastPosition;
    public CalibrationMap motorCal = new CalibrationMap(ClimberConstants.kCalibrationX, ClimberConstants.kCalibrationY);
    public DutyCycleEncoder m_angleSensor;
    public NTDouble nt_targetPosition = new NTDouble(0, table, "target", null);
    public NTDouble nt_position = new NTDouble(0, table, "position", null);
    DoublePublisher nt_positionSensor = table.getDoubleTopic("positionSensor").publish();
    DoublePublisher nt_currentAngle = table.getDoubleTopic("currentAngle").publish();
    BooleanPublisher nt_climberCalibrated = table.getBooleanTopic("climberCalibrated").publish();
    BooleanPublisher nt_hasReachedInPos = table.getBooleanTopic("hasReachedInPos").publish();
    BooleanPublisher nt_hasReachedOutPos = table.getBooleanTopic("hasReachedOutPos").publish();
    BooleanPublisher nt_climbMotorMoving = table.getBooleanTopic("climbMotorMoving").publish();
    BooleanPublisher nt_posIsUpdating = table.getBooleanTopic("posIsUpdating").publish();
    NTDouble nt_relativePosition = new NTDouble(0, table, "relativePosition", (val)->setRelativePosition(Units.degreesToRadians(val)));
    {nt_relativePosition.resetOnRecv = true;}

    public boolean atTargetAngle(){ 
        return Math.abs(m_motor.getPosition()-m_motor.getTargetPosition()) < ClimberConstants.kMotorTolerance;
    }

    /**
     * Returns the position of the climber motor
     * @return in rotations
     */
    public double getPosition(){
        return m_motor.getPosition();
    }

    public boolean posIsUpdating() {
        double currentPosition = m_motor.getPosition();
        boolean isChanging = (currentPosition != lastPosition);
        lastPosition = currentPosition; // Update lastPosition after checking
        return isChanging;
    }

    /**
     * Moves the climber from the current position by a value.
     * @param position in radians
     */
    public void setRelativePosition(double radians){
        m_motor.setRelativePosition(Units.radiansToRotations(radians));
    }
    
    public Command ClimberInCommand = Commands.sequence(
        Commands.repeatingSequence(
            new InstantCommand(() -> m_motor.setRelativePosition(ClimberConstants.kRotationInLead*0.020))
        ).until(()->motorCal.get(m_angleSensor.get()) <= Units.radiansToRotations(ClimberConstants.kAngleIn)),
            new InstantCommand(() -> hasReachedInPos = true),
            new InstantCommand(() -> hasReachedOutPos = false)
    );
    
    public Command ClimberOutCommand = Commands.sequence(
        Commands.repeatingSequence(
            new InstantCommand(() -> m_motor.setRelativePosition(ClimberConstants.kRotationOutLead*0.020))
        ).until(()->motorCal.get(m_angleSensor.get()) >= Units.radiansToRotations(ClimberConstants.kAngleOut)),
            new InstantCommand(() -> hasReachedOutPos = true),
            new InstantCommand(() -> hasReachedInPos = false)
    );
    
    public Climber(int CANID, int analogID) {

        m_angleSensor = new DutyCycleEncoder(analogID);

        m_motor = new Motor(ClimberConstants.kCANID, ClimberConstants.kMotorType, "climber")
            .pidf(ClimberConstants.kPositionGain, GainSlot.POSITION)
            .withStatorLimit(ClimberConstants.kMotorCurrentLimit)
            .inverted(false)
            .idleBrake(true)
            .withGearRatio((ClimberConstants.kGearRatio))
            .withSpeedLimit(ClimberConstants.kMaxSpeed);
    }

    @Override
    public void periodic() {
        nt_targetPosition.set(m_motor.getTargetPosition());
        nt_position.set(m_motor.getPosition());
        nt_positionSensor.set(m_angleSensor.get());
        nt_currentAngle.set(Units.rotationsToDegrees(motorCal.get(m_angleSensor.get())));
        nt_climberCalibrated.set(m_motor.isSetupDone());
        nt_hasReachedInPos.set(hasReachedInPos);
        nt_hasReachedOutPos.set(hasReachedOutPos);
        nt_posIsUpdating.set(posIsUpdating());
    }
}
