package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EffectorConstants;
import frc.robot.Robot;
import frc.utils.Motor;
import frc.utils.NTValues.NTBoolean;

public class EndEffector extends SubsystemBase{

    private static EndEffector instance = new EndEffector(EffectorConstants.kEffectorCANID);

    public static EndEffector getInstance(){
        return instance;
    }
    
    public Motor m_motorEffector;
    Command coralCommand;

    NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/CAUTION/TestInput");

    public EndEffector(int CANID) {
      m_motorEffector = new Motor(Constants.EffectorConstants.kEffectorCANID, Motor.MyMotorType.NEO, "effector")
          .smartCurrentLimit((int)EffectorConstants.kEffectorMotorCurrentLimit)
          .inverted(true)
          .positionConversionFactor((EffectorConstants.kEffectorEncoderPositionFactor))
          .pidf(EffectorConstants.kEffectorKp, EffectorConstants.kEffectorKi, EffectorConstants.kEffectorKd, EffectorConstants.kEffectorKff);
    }

    public DigitalInput m_beamBreak = new DigitalInput(EffectorConstants.kBeamBreakPin);
    public NTBoolean m_beamBreakTestSensor = (Robot.isReal() ? null : new NTBoolean(true, testtable, "Intake/BeamBreakTestInput", (val)->{}));

    private boolean m_beamBroken = false;
    public Debouncer m_beamDebouncer = new Debouncer(EffectorConstants.kBeamBreakDebounceSec, Debouncer.DebounceType.kBoth);

    coralCommand = Commands.sequence(
      Commands.waitUntil(() -> {
            m_motorEffector.setSpeed(EffectorConstants.kEffectorSpeed);
        }
      )
    );

    @Override
    public void periodic() {

    boolean beam_trigger = !m_beamBroken; {
    if (m_beamBreakTestSensor != null){
      m_beamBroken = m_beamDebouncer.calculate(!m_beamBreakTestSensor.get());
    } else {
      m_beamBroken = m_beamDebouncer.calculate(!m_beamBreak.get());
    }
    // rising edge trigger on beam break sensor
    beam_trigger = beam_trigger && m_beamBroken;

    if (beam_trigger){
        // TODO: Initialize the action (e.g., handling the gamepiece).
        // TODO: Schedule the action to be executed.
            }
        }
    }
}
