package frc.utils;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Laser extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Lasers");

    int CANID;
    MyLaserType laserType_;
    String name_;

    private LaserCan lc;

    DoublePublisher nt_currentDistance;

    public Laser (int CANID_, MyLaserType laserType_, String name_) {
        this.CANID = CANID_;
        this.laserType_ = laserType_;
        this.name_ = name_;

        if (laserType_ == MyLaserType.LASERCAN) {
            lc = new LaserCan(CANID_);
        }

        // Name will be used to distinguish lasers when we integrate the Fusion laser
        nt_currentDistance = table.getDoubleTopic(name_ + "_Distance").publish();

    }

    public enum MyLaserType {
        LASERCAN, FUSIONLASER, SIMULATED
    }

       public Laser setRangingMode(/* Figure out how to represent SHORT and LONG */){
        switch (laserType_) {
            case LASERCAN:
                try {
                    lc.setRangingMode(LaserCan.RangingMode.SHORT);
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed! " + e);
                }
                break;
            case FUSIONLASER:
                // Add this
                break;
            default:
        }
        return this;
    }

    public Laser setRegionOfInterest(/* Figure out how to represent Region of Interest */){
        switch (laserType_) {
            case LASERCAN:
                try {
                lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed! " + e);
                }
                break;
            case FUSIONLASER:
                // Add this
                break;
            default:
        }
        return this;
    }

    public Laser setTimingBudget(/* Figure out how to represent Timing Budget */){
        switch (laserType_) {
            case LASERCAN:
                try {
                lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed! " + e);
                }
                break;
            case FUSIONLASER:
                // Add this
                break;
            default:
        }
        return this;
    }

    @Override
    public void periodic(){
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
          System.out.println("The target is " + measurement.distance_mm + "mm away!");

          // Converting mm to meters, then meters to inches
          double meters = measurement.distance_mm / 1000.0;
          double inches = Units.metersToInches(meters);
          nt_currentDistance.set(inches);

        } else {
          System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
          nt_currentDistance.set(0.0);
        }

    }
    
}
