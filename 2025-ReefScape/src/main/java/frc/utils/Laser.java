package frc.utils;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;

public class Laser extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Lasers");

    // Region of Interest presets [topLeftX, topLeftY, bottomRightX, bottomRightY]
    public static final int[] ROI_FULL = {0, 0, 15, 15};      // Full 16x16 sensor, widest FOV
    public static final int[] ROI_CENTER = {6, 6, 10, 10};    // 4x4 center region
    public static final int[] ROI_NARROW = {7, 7, 9, 9};      // 2x2 center region, narrowest FOV

    // Sample time presets for fusion sensor only (24-1000ms)
    public static final double SAMPLE_TIME_FAST = 24.0;
    public static final double SAMPLE_TIME_BALANCED = 50.0;
    public static final double SAMPLE_TIME_ACCURATE = 100.0;

    int CANID;
    MyLaserType laserType_;
    String name_;

    private LaserCan lc;
    private TimeOfFlight fusionTof;

    DoublePublisher nt_currentThriftyDistance, nt_currentFusionDistance;

    public Laser (int CANID_, MyLaserType laserType_, String name_) {
        this.CANID = CANID_;
        this.laserType_ = laserType_;
        this.name_ = name_;

        if (laserType_ == MyLaserType.LASERCAN) {
            lc = new LaserCan(CANID_);
        } else if (laserType_ == MyLaserType.FUSIONLASER) {
            fusionTof = new TimeOfFlight(CANID_);
        }

        nt_currentThriftyDistance = table.getDoubleTopic("LaserCanSensorDistance").publish();
        nt_currentFusionDistance = table.getDoubleTopic("FusionSensorDistance").publish();

        // Set default configuration
        // LaserCan has predefined sample times so it ignores the argument
        setRangingMode(MyRangingMode.LONG, SAMPLE_TIME_BALANCED);
        setRegionOfInterest(ROI_NARROW);

    }

    public enum MyLaserType {
        LASERCAN, FUSIONLASER, SIMULATED
    }

    public enum MyRangingMode {
        SHORT, MEDIUM, LONG
    }

       public Laser setRangingMode(MyRangingMode mode, double sampleTimeMs){
        switch (laserType_) {
            case LASERCAN:
                try {
                    // LaserCan only supports SHORT/LONG
                    lc.setRangingMode(mode == MyRangingMode.LONG ?
                    LaserCan.RangingMode.LONG : LaserCan.RangingMode.SHORT);
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed! " + e);
                }
                break;
            case FUSIONLASER:
                TimeOfFlight.RangingMode fusionMode;
                switch (mode) {
                    case SHORT:
                        fusionMode = TimeOfFlight.RangingMode.Short;
                        break;
                    case MEDIUM:
                        fusionMode = TimeOfFlight.RangingMode.Medium;
                        break;
                    case LONG:
                        fusionMode = TimeOfFlight.RangingMode.Long;
                        break;
                    default:
                        fusionMode = TimeOfFlight.RangingMode.Short;
                }
                fusionTof.setRangingMode(fusionMode, sampleTimeMs);
                break;
            default:
        }
        return this;
    }

    public Laser setRegionOfInterest(int[] roi){
        switch (laserType_) {
            case LASERCAN:
                try {
                lc.setRegionOfInterest(new LaserCan.RegionOfInterest(roi[0], roi[1], roi[2], roi[3]));
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed! " + e);
                }
                break;
            case FUSIONLASER:
                fusionTof.setRangeOfInterest(roi[0], roi[1], roi[2], roi[3]);
                break;
            default:
        }
        return this;
    }

    public Laser setTimingBudget(int budgetMs){
        switch (laserType_) {
            case LASERCAN:
                try {
                    LaserCan.TimingBudget budget;
                    if (budgetMs <= 20) budget = LaserCan.TimingBudget.TIMING_BUDGET_20MS;
                    else if (budgetMs <= 33) budget = LaserCan.TimingBudget.TIMING_BUDGET_33MS;
                    else if (budgetMs <= 50) budget = LaserCan.TimingBudget.TIMING_BUDGET_50MS;
                    else budget = LaserCan.TimingBudget.TIMING_BUDGET_100MS;
                    lc.setTimingBudget(budget);
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed! " + e);
                }
                break;
            case FUSIONLASER:
                // The Fusion sensor's timing is set through RangingMode with sampleTime param.
                // Getting current ranging mode and reapplying it with new sample time.
                TimeOfFlight.RangingMode currentMode = fusionTof.getRangingMode();
                fusionTof.setRangingMode(currentMode, budgetMs);
                break;
            default:
        }
        return this;
    }

    @Override
    public void periodic(){

        switch (laserType_) {
            case LASERCAN:
                LaserCan.Measurement measurement = lc.getMeasurement();
                if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                System.out.println("The LaserCan's target is " + measurement.distance_mm + "mm away!");
                // Converting mm to meters, then meters to inches
                double meters = measurement.distance_mm / 1000.0;
                double inches = Units.metersToInches(meters);
                nt_currentThriftyDistance.set(inches);

                } else {
                System.out.println("Oh no! The LaserCan's target is out of range, or we can't get a reliable measurement!");
                nt_currentThriftyDistance.set(0.0);
                }
                break;
            case FUSIONLASER:
                if (fusionTof.isRangeValid()) {
                    double range = fusionTof.getRange(); // Returns distance in mm
                    System.out.println("The Fusion laser's target is " + range + "mm away!");
                    double meters = range / 1000.0;
                    double inches = Units.metersToInches(meters);
                    nt_currentFusionDistance.set(inches);
                } else {
                    System.out.println("Oh no! The Fusion laser's target is out of range, or we can't get a reliable measurement!");
                    nt_currentFusionDistance.set(0.0);
                }
                break;
            default:
        }
    }
    
}
