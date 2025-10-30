package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Encouragement extends SubsystemBase{

public boolean driveTeamIsEncouraged = false;

    public void driveTeamEncouragement(){
        if (DriverStation.isEnabled() && driveTeamIsEncouraged == false) {
        System.out.println("You can do this Rosie, LETS GOOO!!");
        driveTeamIsEncouraged = true;
        }
    }

    @Override
    public void periodic() {
        driveTeamEncouragement();
    }
}
