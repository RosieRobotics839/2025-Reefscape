package frc.robot;

import com.ctre.phoenix6.controls.MusicTone;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Timer;

public class TestMotorToneCommand extends Command {
    private static final double TEST_FREQUENCY = 1000.0; // A4 note (440Hz)
    private static final double DURATION = 3.0; // Seconds to play the tone
    private final Arm m_arm;
    private final Timer m_timer = new Timer();
    private boolean m_finished = false;

    public TestMotorToneCommand(Arm arm) {
        m_arm = arm;
        addRequirements(arm); // This ensures other commands don't use the arm while playing
    }

    @Override
    public void initialize() {
        System.out.println("Starting tone test...");
        m_timer.reset();
        m_timer.start();
        m_finished = false;

        // Make sure the motor is a Kraken (TalonFX)
        if (m_arm.m_motor.motor_talon != null) {
            // Create a MusicTone control request
            MusicTone musicTone = new MusicTone(TEST_FREQUENCY);
            
            // Optional: set update frequency (how often the tone is refreshed)
            musicTone.UpdateFreqHz = 50.0; // 50Hz update rate
            
            // Send the request to the motor
            m_arm.m_motor.motor_talon.setControl(musicTone);
            System.out.println("Tone request sent to motor");
        } else {
            System.out.println("Error: Arm motor is not a TalonFX (Kraken)");
            m_finished = true;
        }
    }

    @Override
    public void execute() {
        // While running, print time remaining
        if (m_timer.hasElapsed(DURATION)) {
            m_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the tone by sending a zero frequency
        if (m_arm.m_motor.motor_talon != null) {
            m_arm.m_motor.motor_talon.setControl(new MusicTone(0.0));
            System.out.println("Tone stopped");
        }
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
