// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveModule;

public class KrakenOrchestra {
    private Orchestra m_orchestra = new Orchestra();
    private List<TalonFX> motors;

    public KrakenOrchestra() { //idk what im doing and this laptop is about to die - Eli

        _elevator = Elevator.getInstance();
        m_orchestra.addInstrument(_elevator.m_EleMotor);
    }
}