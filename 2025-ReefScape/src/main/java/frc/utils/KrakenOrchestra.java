// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OrchestraConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;


public class KrakenOrchestra extends SubsystemBase {

    private static KrakenOrchestra instance = new KrakenOrchestra();

    public static KrakenOrchestra getInstance(){
        return instance;
    }

    public Orchestra m_orchestra = new Orchestra();

    List<Motor> instruments = new ArrayList<>();

    instruments.add(_arm.m_motor);
    instruments.add(_elevator.m_motor);
    instruments.add(_driveTrain.frontLeft);
    instruments.add(_driveTrain.frontRight);
    instruments.add(_driveTrain.rearLeft);
    instruments.add(_driveTrain.rearRight);

    Arm _arm = Arm.getInstance();
    Elevator _elevator = Elevator.getInstance();
    DriveTrain _driveTrain = DriveTrain.getInstance();

    public KrakenOrchestra(instruments){
        m_orchestra.addInstrument(motor);
    }

    // Playing Music 
    public void playMusic(String filepath){
        var status = m_orchestra.loadMusic(filepath);
        if (status.isOK()){
            m_orchestra.play();
        } 
    }

    public void stopMusic(){
        m_orchestra.stop();
    }

    public boolean isPlaying(){
        return m_orchestra.isPlaying();
    }

}