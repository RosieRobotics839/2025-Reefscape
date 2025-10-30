package frc.utils;

import frc.robot.Constants;
import frc.utils.Motor.MyMotorType;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Laser extends SubsystemBase {

    int CANID;
    MyLaserType laserType;
    String name;

    private LaserCan lc;
    
    lc = new LaserCan(0);

    public enum MyLaserType {
        LASERCAN, FUSIONLASER, SIMULATED
    }

       public Laser setRangingMode(/* Figure out how to represent SHORT and LONG */){
        switch (laserType) {
            case LASERCAN:
                lc.setRangingMode(LaserCan.RangingMode.SHORT);
                break;
            case FUSIONLASER:
                // Add this
                break;
            default:
        }
        return this;
    }

    public Laser setRegionOfInterest(/* Figure out how to represent Region of Interest */){
        switch (laserType) {
            case LASERCAN:
                lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
                break;
            case FUSIONLASER:
                // Add this
                break;
            default:
        }
        return this;
    }

    public Laser setTimingBudget(/* Figure out how to represent Timing Budget */){
        switch (laserType) {
            case LASERCAN:
                lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
                break;
            case FUSIONLASER:
                // Add this
                break;
            default:
        }
        return this;
    }
    
}
