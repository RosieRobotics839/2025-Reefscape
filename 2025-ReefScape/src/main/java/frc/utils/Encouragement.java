package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Encouragement extends SubsystemBase{

    public void driveTeamEncouragement(){
        if (DriverStation.isEnabled()) {
        System.out.println("You can do this Rosie, LETS GOOO!!");
        }
    }

    @Override
    public void periodic() {
        driveTeamEncouragement();
    }
}
