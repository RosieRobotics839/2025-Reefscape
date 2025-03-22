// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class KrakenOrchestra extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Orchestra");

    private static KrakenOrchestra instance = new KrakenOrchestra();

    public static KrakenOrchestra getInstance(){
        return instance;
    }

    public Orchestra m_orchestra = new Orchestra();

    private List<TalonFX> instruments = new ArrayList<>();
    private String currentSong = "";
    private StatusCode status;
    private boolean orchestraReady = false;
    BooleanPublisher nt_orchestraIsPlaying = table.getBooleanTopic("orchestraIsPlaying").publish();
    BooleanPublisher nt_orchestraIsReady = table.getBooleanTopic("orchestraIsReady").publish();
    IntegerPublisher nt_instrumentCount = table.getIntegerTopic("instrumentCount").publish();
    StringPublisher nt_currentFile = table.getStringTopic("currentSongPlaying").publish();

    private KrakenOrchestra() {
        // Initialize orchestra with all Kraken motors on the robot
        initializeInstruments();
    }

    private void initializeInstruments() {
        // Clear any existing instruments
        instruments.clear();
        
        // Add Kraken motors from our subsystems
        try {
            // Only add motors that are TalonFX (Kraken) instances
            if (Arm.getInstance().m_motor.motor_talon != null) {
                instruments.add(Arm.getInstance().m_motor.motor_talon);
            }
            
            if (Elevator.getInstance().m_EleMotor.motor_talon != null) {
                instruments.add(Elevator.getInstance().m_EleMotor.motor_talon);
            }
            
            DriveTrain dt = DriveTrain.getInstance();
            if (dt.frontLeft.m_motorDrive.motor_talon != null) instruments.add(dt.frontLeft.m_motorDrive.motor_talon);
            if (dt.frontRight.m_motorDrive.motor_talon != null) instruments.add(dt.frontRight.m_motorDrive.motor_talon);
            if (dt.rearLeft.m_motorDrive.motor_talon != null) instruments.add(dt.rearLeft.m_motorDrive.motor_talon);
            if (dt.rearRight.m_motorDrive.motor_talon != null) instruments.add(dt.rearRight.m_motorDrive.motor_talon);
            
            // Add each instrument to the orchestra and allow it to play music when Driver Station is Disabled
            int successCount = 0;
            for (TalonFX motor : instruments) {
                StatusCode addStatus = m_orchestra.addInstrument(motor);
                // Create audio config
                var audioConfig = new com.ctre.phoenix6.configs.AudioConfigs();
                // Set AllowMusicDurDisable to true
                audioConfig.withAllowMusicDurDisable(true);
                // Apply the configuration to the motor
                motor.getConfigurator().apply(audioConfig);

                if (addStatus.isOK()) {
                    successCount++;
                }
            }
            orchestraReady = successCount > 0;
        } catch (Exception e) {
            orchestraReady = false;
        }
    }

    // Playing Music 
    public void playMusic(String filepath) {
        if (filepath == null || filepath.isEmpty()) {
            return;
        }
        
        // If we're already playing this song, don't restart it
        if (isPlaying() && filepath.equals(currentSong)) {
            return;
        }
        
        if (!orchestraReady) {
            initializeInstruments();
            return;
        }

        currentSong = filepath;
        stopMusic(); // Stop any currently playing music
        
        status = m_orchestra.loadMusic(filepath);
        if (status.isOK()) {
            m_orchestra.play();
            System.out.println("Playing music: " + filepath);
        } else {
            System.out.println("Failed to load music: " + filepath);
            // Driver Station Log flows too fast to catch this
        }
    }

    public void stopMusic(){
        if (orchestraReady) {
            m_orchestra.stop();
        }
    }

    public boolean isPlaying(){
        return m_orchestra.isPlaying() && orchestraReady;
    }

    public boolean isReady() {
        return orchestraReady;
    }

    @Override
    public void periodic() {

        nt_orchestraIsPlaying.set(isPlaying());
        nt_orchestraIsReady.set(orchestraReady);
        nt_instrumentCount.set(instruments.size());
        nt_currentFile.set(currentSong);

        // If motors have been initialized since our last check, try initializing instruments again
        if (!orchestraReady) {
            // Check if the motors we need are now available
            boolean motorsAvailable = false;
            
            if (Arm.getInstance().m_motor.isSetupDone() && 
                Elevator.getInstance().m_EleMotor.isSetupDone() &&
                DriveTrain.getInstance().isSetupDone()) {
                
                motorsAvailable = true;
            }
            
            if (motorsAvailable) {
                initializeInstruments();
            }
        }

    }

}