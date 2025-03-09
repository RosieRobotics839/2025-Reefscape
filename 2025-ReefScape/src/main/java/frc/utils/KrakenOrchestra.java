// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OrchestraConstants;


public class KrakenOrchestra extends SubsystemBase {
    public Orchestra m_orchestra = new Orchestra();

    public KrakenOrchestra(TalonFX motor){
        m_orchestra.addInstrument(motor);
    }

    public void playMusic(){
        var status = m_orchestra.loadMusic(OrchestraConstants.getSongSelection());
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