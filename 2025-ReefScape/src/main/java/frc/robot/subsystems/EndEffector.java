package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EffectorConstants;
import frc.robot.Robot;
import frc.utils.Motor;
import frc.utils.NTValues.NTBoolean;

public class EndEffector extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/EndEffector");

    private static EndEffector instance = new EndEffector(EffectorConstants.kCANID);

    public static EndEffector getInstance(){
        return instance;
    }
    
    public Motor m_motor;

    public boolean m_intakeRunning = false;

    private boolean m_beamBroken = false;
    private boolean m_hasGamePiece = false;
    private boolean m_hasCoral = false;
    private double m_algaeRelativePosition;

    public Debouncer m_beamDebouncer = new Debouncer(EffectorConstants.kBeamBreakDebounceSec, Debouncer.DebounceType.kBoth);

    public boolean hasGamePiece(){
      return m_hasGamePiece;
    }

    Command IntakeCommand = Commands.sequence(
      Commands.waitUntil(() -> {return m_motor.setSpeed(EffectorConstants.kIntakeSpeed);}), //Sets speed to intake game piece
      Commands.waitUntil(() -> {return hasGamePiece();}), //Checking whether we have a game piece or not.
      new InstantCommand(() -> m_motor.setRelativePosition(0.1))
    )
    .beforeStarting(()->m_intakeRunning=true)
    .finallyDo(()->{m_motor.setSpeed(0); m_intakeRunning=false;});

    public Command ExpelCommand(DoubleSupplier speed, BooleanSupplier extended){
      return Commands.sequence( //Outtake for those who don't know
        Commands.waitUntil(() -> {return m_motor.setSpeed((m_hasCoral ? 1 : -1) * speed.getAsDouble());}), // If we have the coral ( ? ) then forward, anything else backward.
        Commands.waitUntil(() -> {return !hasGamePiece();}), //Checking whether we have a game piece or not.
        Commands.waitSeconds(3).onlyIf(extended)
      ).beforeStarting(()->m_intakeRunning=false)
      .finallyDo(()->m_motor.setSpeed(0));
    };

    public EndEffector(int CANID) {

      m_motor = new Motor(Constants.EffectorConstants.kCANID, EffectorConstants.kMotorType, "effector")
          .withStatorLimit((int)EffectorConstants.kMotorCurrentLimit)
          .inverted(false)
          .withGearRatio((EffectorConstants.kGearRatio))
          .withSpeedLimit(EffectorConstants.kOuttakeSpeed)
          .withSlowSpeedControl(true)
          .pidf(EffectorConstants.kGainPosition, Motor.GainSlot.POSITION)
          .pidf(EffectorConstants.kGainVelocity, Motor.GainSlot.SPEED);
    }

    public DigitalInput m_beamBreak = new DigitalInput(EffectorConstants.kBeamBreakPin);
    public NTBoolean m_beamBreakTestSensor = (Robot.isReal() ? null : new NTBoolean(true, table, "Effector/BeamBreakTestInput", (val)->{}));
    public NTBoolean nt_beamBroken = new NTBoolean(false, table, "Effector/beamBroken", (val) -> {});
    public NTBoolean nt_hasGamePiece = new NTBoolean(false, table, "Effector/hasGamePiece", (val) -> {});
    public NTBoolean nt_hasCoral = new NTBoolean(false, table, "Effector/hasCoral", (val) -> {});

    @Override
    public void periodic() {

    boolean beam_trigger = !m_beamBroken;
    if (m_beamBreakTestSensor != null){
      m_beamBroken = m_beamDebouncer.calculate(!m_beamBreakTestSensor.get());
    } else {
      m_beamBroken = m_beamDebouncer.calculate(!m_beamBreak.get());
    }

    nt_beamBroken.set(m_beamBroken);
    nt_hasGamePiece.set(m_hasGamePiece);
    nt_hasCoral.set(m_hasCoral);

    // rising edge trigger on beam break sensor
    beam_trigger = beam_trigger && m_beamBroken;

    // Checking to see if we have coral
    if (beam_trigger){
        m_hasGamePiece = true;
        m_hasCoral = true;
    }
    
    // Coral not detected, checking to see if we have algae.
  /* if (!m_beamBroken && (m_motor.getVelocity() < .1 || m_motor.getOutputCurrent() > 10)){
      m_hasGamePiece = true; 
      m_hasCoral = false;
      m_algaeRelativePosition = m_motor.getPosition();
    //Checking to see whether we have a game piece or not by checking if the beam is broken, or velocity is less than 10 RPM, or Current is greater than 40
    } */

    if (m_hasCoral && !m_beamBroken){ // Checking to see if we previously had coral. Seeing is m_hasCoral is still correct because if the beam is not broken we don't have coral anymore.
      m_hasGamePiece = false;
      m_hasCoral = false;
    }

    if (m_hasGamePiece && !m_hasCoral && m_motor.getPosition() < (m_algaeRelativePosition - EffectorConstants.kAlgaeMotorRevolutions)){ //Checking to see if we still have algae by waiting until the position gets to a certain point.
      m_hasGamePiece = false;
    }
  }
}
