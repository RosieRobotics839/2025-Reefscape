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
import edu.wpi.first.networktables.IntegerPublisher;
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
            
            // Add each instrument to the orchestra
            int successCount = 0;
            for (TalonFX motor : instruments) {
                StatusCode addStatus = m_orchestra.addInstrument(motor);
                if (addStatus.isOK()) {
                    successCount++;
                }
            }
            orchestraReady = successCount > 0;
        } catch (Exception e) {
            // System.out.println("Error initializing orchestra instruments: " + e.getMessage());
            // Driver Station Log flows too fast to catch this
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
        } else {
            // System.out.println("Failed to load music: " + status.toString());
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

    }

}