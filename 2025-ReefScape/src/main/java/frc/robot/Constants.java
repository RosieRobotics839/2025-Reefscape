// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.utils.Motor;
import frc.utils.Motor.MyMotorType;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;
import frc.utils.NTValues.NTInteger;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  // TYPE DEFINITIONS
  public static class CANID_t {
    public int encoder, driving, steering;
    public CANID_t(int abs_encoder, int drive, int steer){this.encoder = abs_encoder; this.driving = drive; this.steering = steer;}
  }

  public static class ScoreConstants {

    public enum ScoreLevel{
      FUNNEL, TROUGH, CORAL2, CORAL3, CORAL4, ALGAE0, ALGAE1, ALGAE2, ALGAE3, ALGAE4
    }

    public enum ReefAlignment {
      LEFT, CENTER, RIGHT
    }
    
  }


  public static class MotorDefaults {
    public static class Kraken {
      public static Motor.Gains kGainPosition = new Motor.Gains(1.2,2.0,0,0);
      public static Motor.Gains kGainSpeed = new Motor.Gains(0.2,0.0,0.0,0.115);
      public static Motor.Gains kGainAux1 = new Motor.Gains(0,0,0,0);
      public static Motor.Gains kGainAux2 = new Motor.Gains(0,0,0,0);
      public static double kPositionGainRatio = kGainPosition.Kp/kGainSpeed.Kp; // For SLOWSPEED control
    }
    public static class NEO {
      public static Motor.Gains kGainPosition = new Motor.Gains(.05,0.00024,0,0);
      public static Motor.Gains kGainSpeed = new Motor.Gains(0.015,0.0,0.0,0.01100);
      public static Motor.Gains kGainAux1 = new Motor.Gains(0,0,0,0);
      public static Motor.Gains kGainAux2 = new Motor.Gains(0,0,0,0);
      public static double kPositionGainRatio = kGainPosition.Kp/kGainSpeed.Kp; // For SLOWSPEED control
    }

    public static double kOutputRange = 1;
    public static double iZone = 0.2;
    public static int kCurrentLimit = 5;
    public static boolean kInverted = false;
    public static Boolean kIdleBrake = false;
  
    // If motor rotations per second less than kSlowThreshold, switches to position control for precise slow speed movement.
    public static double kSlowThreshold = NTDouble.create(2.5, "MotorDefault/kSlowThreshold", (val)->kSlowThreshold = val);
    public static double kSlowHysteresis = NTDouble.create(.2, "MotorDefault/kSlowHysteresis", (val)->kSlowHysteresis = val);
    public static double kSlowTransitionExtraSpin = NTDouble.create(.7,"MotorDfault/kSlowTransExtraspin",(val)->kSlowTransitionExtraSpin=val);

  }

  public static class OperatorConstants {
    // Controller Input Settings
    public static final int kDriverControllerPort = 0;
    public static final int kAccessoryControllerPort = 1;
    public static boolean kDriverControllerIsFlightStick = NTBoolean.create(true, "Operator/DriverControllerIsFlightStick",val->kDriverControllerIsFlightStick=val);

    // Default Settings
    public static boolean kFieldCentricDriving = true;
    public static boolean kDefaultDriveAccelLimiter = true;
    public static double kControllerActiveThreshold = 0.4;
    public static double kControllerActiveHysteresis = 0.1;
  }


  public static class GyroConstants{
    public static boolean kEnabled = true;
    public static double kTippingAngle = Units.degreesToRadians(NTDouble.create(12.5,"Gyro/kTippingAngle",val->kTippingAngle=Units.degreesToRadians(val)));
    public static double kTippingHysteresis = Units.degreesToRadians(NTDouble.create(5,"Gyro/kTippingHysteresis",val->kTippingHysteresis=Units.degreesToRadians(val)));
    public static int kCANID = 50;
    public static double kVisionCorrectionMaxRate = Units.degreesToRadians(NTDouble.create(40,"Gyro/kVisionCorrectionMaxRate",val->kVisionCorrectionMaxRate=Units.degreesToRadians(val)));
  }

  public static class PoseConstants{
    public static double kVisionWeightRotDecay = NTDouble.create(0.99, "Pose/kVisionWeightRotDecay", val->kVisionWeightRotDecay=val);
    public static double kVisionWeightRot = NTDouble.create(0.05, "Pose/kVisionWeightTheta", val->kVisionWeightRot=val);
    public static double kVisionWeightPos = NTDouble.create(0.2, "Pose/kVisionWeightPos", val->kVisionWeightPos=val);
    public static double kGyroWeight = NTDouble.create(0.10, "Pose/kGyroWeight",val->kGyroWeight=val);
    public static double kDriveSlip = NTDouble.create(1.0, "Pose/kDriveSlip",val->kDriveSlip=val);;
  }

  public static class VisionConstants{
    public static double kMaxAmbiguity = NTDouble.create(0.2, "Vision/kMaxAmbiguity", val->kMaxAmbiguity=val);
    public static int    kPipelineIndex = NTInteger.create(0, "Vision/kPipelineIndex", val->Vision.cam1.setPipelineIndex(val));
    public static double kMinTargetArea = NTDouble.create(.1, "Vision/kMinTargetArea", val->kMinTargetArea=val);
    
    // Network Table boolean for switching between field layouts
    public static boolean isChampionshipGame = NTBoolean.create(false, "isChampionshipGame", val -> {isChampionshipGame = val; Vision.getInstance().reloadFieldLayout();});
    // Method to get the current field layout path
    public static String getFieldLayoutPath() {
      //return Filesystem.getDeployDirectory() + "/" + "2025-reefscape-andymark.json";
      //return Filesystem.getDeployDirectory() + "/" + "2025-reefscape-welded.json";
      //return Filesystem.getDeployDirectory() + "/" + "2025-reefscape-buzz.json";
      //return Filesystem.getDeployDirectory() + "/" + "2025-waterbury-practice-field.json";
      return Filesystem.getDeployDirectory() + "/" + "2025-rosiecarpet.json";
    }

    public static String kFieldLayout = getFieldLayoutPath();

    public static double kMaxLatencyCompensationMillis = 200; // ms

    public static double kPixyTimeout = 1; // second


    // Simulated Vision System
    public static class frontCamera{
      public static final String kCameraName = "FrontCam";

      public static double kCamYawRight = Math.toRadians(NTDouble.create(15, "Vision/"+kCameraName+"/kCamYawRight_deg", val->{kCamYawRight=Math.toRadians(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamPitchUp = Math.toRadians(NTDouble.create(0, "Vision/"+kCameraName+"/kCamPitchUp_deg", val->{kCamPitchUp=Math.toRadians(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamDiagFOV = Math.toRadians(NTDouble.create(77.2, "Vision/"+kCameraName+"/kCamDiagFOV_deg", val->{kCamDiagFOV=Math.toRadians(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamForwardOffset = Units.inchesToMeters(NTDouble.create(6.875, "Vision/"+kCameraName+"/kCamForwardOffset_in", val->{kCamForwardOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamLeftOffset = Units.inchesToMeters(NTDouble.create(7.375 + 4 /* calibrated offset */, "Vision/"+kCameraName+"/kCamLeftOffset_in", val->{kCamLeftOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));

      public static double kSimMaxLEDRange = 20; // meters
      public static int    kSimCamResolutionW = 1280; // pixels
      public static int    kSimCamResolutionH = 720; // pixels
      
      public static double kCamHeightOffGround = Units.inchesToMeters(14.25);
      public static Transform3d kCameraToRobot(){
        return new Transform3d(
          new Translation3d(
            -kCamForwardOffset,
            -kCamLeftOffset,
            -kCamHeightOffGround
          ),
          new Rotation3d(0.0, kCamPitchUp, kCamYawRight)
        );
      }
    }

    public static class rearCamera{
      public static final String kCameraName = "RearCam";
    
      public static double kCamYawRight = Math.toRadians(NTDouble.create(180, "Vision/"+kCameraName+"/kCamYawRight_deg", val->{kCamYawRight=Math.toRadians(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamPitchUp = Math.toRadians(NTDouble.create(-5, "Vision/"+kCameraName+"/kCamPitchUp_deg", val->{kCamPitchUp=Math.toRadians(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamDiagFOV = Math.toRadians(NTDouble.create(76.06, "Vision/"+kCameraName+"/kCamDiagFOV_deg", val->{kCamDiagFOV=Math.toRadians(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamForwardOffset = Units.inchesToMeters(NTDouble.create(0, "Vision/"+kCameraName+"/kCamForwardOffset_in", val->{kCamForwardOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamRightOffset = Units.inchesToMeters(NTDouble.create(2, "Vision/"+kCameraName+"/kCamLeftOffset_in", val->{kCamRightOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));

      public static double kSimMaxLEDRange = 20; // meters
      public static int    kSimCamResolutionW = 1280; // pixels
      public static int    kSimCamResolutionH = 720; // pixels
      
      public static double kCamHeightOffGround = Units.inchesToMeters(39);
      public static Transform3d kCameraToRobot(){
        return new Transform3d(
          new Translation3d(
            -kCamForwardOffset,
            -kCamRightOffset,
            -kCamHeightOffGround
          ),
          new Rotation3d(0.0, kCamPitchUp, kCamYawRight)
        );
      }
    }
  }

  // Chassis configuration
  public static class kChassis {
    public static final double kTrackWidth = Units.inchesToMeters(24.25); // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(24.25); // Distance between front and back wheels
  }
  
  public static class AutoConstants {
    
    public static final double kFieldReefCenterFromAprilTagDistance = 0.831723; // Determined from Andymark Apriltag Map
    public static final int kReefRedCenterRefID = 7;
    public static final int kReefBlueCenterRefID = 18;
    public static double kReefKOZRadius = Units.feetToMeters(NTDouble.create(6,"Autonomous/kReefKOZRadius", val -> {kReefKOZRadius = Units.feetToMeters(val); Autonomous.staticObstacles = Autonomous.generateStaticObstacles();}));
    public static double kReefGraphNodeRadius = Units.feetToMeters(NTDouble.create(7.0,"Autonomous/kReefGraphNodeRadius", val -> {kReefGraphNodeRadius = Units.feetToMeters(val);}));
    public static double kMinObstaclePenaltyDistance = Units.feetToMeters(NTDouble.create(2.0,"Autonomous/kMinObstaclePenaltyDistance", val -> {kMinObstaclePenaltyDistance = Units.feetToMeters(val);}));
 
    public static double kLineupTimeout = NTDouble.create(7, "Autonomous/kLineupTimeout", val -> kLineupTimeout = val);
    public static double kReefDistance = Units.inchesToMeters(NTDouble.create(6.25, "Autonomous/kReefDistance", val -> kReefDistance = Units.inchesToMeters(val)));
    public static double kReefDistanceCenterAlign = Units.inchesToMeters(NTDouble.create(2.5, "Autonomous/kCenterReefDistance", val -> kReefDistanceCenterAlign = Units.inchesToMeters(val)));
    public static double kReefStartingDistance = Units.inchesToMeters(NTDouble.create(52, "Autonomous/kReefStartingDistance", val -> kReefStartingDistance = Units.inchesToMeters(val)));
    public static double kReefTolerance = Units.inchesToMeters(NTDouble.create(1, "Autonomous/kReefToleranceInch", val -> kReefTolerance = Units.inchesToMeters(val)));
    public static double kReefArmupTolerance = Units.inchesToMeters(NTDouble.create(120, "Autonomous/kReefArmupTolerance", val -> kReefArmupTolerance = Units.inchesToMeters(val)));
    public static double kSourceDistance = Units.inchesToMeters(NTDouble.create(0, "Autonomous/kSourceDistanceInch", val -> kSourceDistance = Units.inchesToMeters(val)));
    public static double kSourceOffset = Units.inchesToMeters(NTDouble.create(14, "Autonomous/kSourceOffsetInch", val -> kSourceOffset = Units.inchesToMeters(val)));
    public static double kSourceStartingDistance = Units.inchesToMeters(NTDouble.create(30.0, "Autonomous/kSourceStartingDistance", val -> kSourceStartingDistance = Units.inchesToMeters(val)));
    public static double kSourceTolerance = Units.inchesToMeters(NTDouble.create(12.0, "Autonomous/kSourceNearDistanceInch", val -> kSourceTolerance = Units.inchesToMeters(val)));
    public static double kReefOffset = Units.inchesToMeters(NTDouble.create(6.5,"Autonomous/kReefOffset",val -> kReefOffset = Units.inchesToMeters(val)));
    public static double kTroughClearance = Units.inchesToMeters(NTDouble.create(2, "Autonomous/kTroughClearance", val -> kTroughClearance = Units.inchesToMeters(val)));
    public static double kBargeDistance = Units.inchesToMeters(NTDouble.create(0.0, "Autonomous/kBargeDistance", val -> kBargeDistance = Units.inchesToMeters(val)));
    public static double kBargeOffset = Units.inchesToMeters(NTDouble.create(30, "Autonomous/kBargeOffset", val -> kBargeOffset = Units.inchesToMeters(val)));
    // Ooga Booga Number
    public static double kStaticReefOffset = Units.inchesToMeters(NTDouble.create(0,"Autonomous/kStaticReefOffset",val -> kStaticReefOffset = Units.inchesToMeters(val)));
    
    public static double kFieldLength = 16.451;
    public static double kFieldWidth = 8.211;
   
  }

  public static class kDriveTrain {
    
    public static class DriveConstants {
      public static double kLinearAccelerationTau = NTDouble.create(0.005, "DriveConstants/kLinearAccelerationTau", val->DriveTrain.forEachSwerveModule((m)->m.m_magLimiter.tau=val));

      public static Integer kMaxSpeedDefault = 0; // 0 indexed selector for arrays below

      public static double [] kMaxSpeedMetersPerSecond = { // m/s
        Units.feetToMeters(5),
        Units.feetToMeters(10),
        Units.feetToMeters(15)
      };

      public static double [] kMaxRotationVelocity = { // rad/s
        Units.degreesToRadians(90), 
        Units.degreesToRadians(90),
        Units.degreesToRadians(90)
      };
      public static double kMaxSpeed = Units.feetToMeters(NTDouble.create(4, "DriveConstants/MaxSpeed",val -> DriveTrain.getInstance().setMaxSpeed(Units.feetToMeters(val))));
      public static boolean kAutoTurnToBestTag = NTBoolean.create(false,"DriveConstants/kAuto/TurnToBestTag",val->kAutoTurnToBestTag=val);
      public static double kAutoSpeedScale = NTDouble.create(1, "DriveConstants/kAuto/SpeedScale", val->kAutoSpeedScale=val);
      public static double kAutoMaxSpeed = Units.feetToMeters(NTDouble.create(12,"DriveConstants/kAuto/MaxSpeedFPS", val->kAutoMaxSpeed = Units.feetToMeters(val)));
      public static double kAutoMinSpeed = Units.feetToMeters(NTDouble.create(2.7,"DriveConstants/kAuto/MinSpeedFPS", val->kAutoMinSpeed = Units.feetToMeters(val)));
      public static double kAutoMinDistance = Units.inchesToMeters(NTDouble.create(6, "DriveConstants/kAuto/MinSpeedDistance", val->kAutoMinDistance=Units.inchesToMeters(val)));
      public static double kAutoAccelLimiter = Units.feetToMeters(NTDouble.create(25,"DriveConstants/kAuto/AccelLimiterFPS2", val->DriveTrain.getInstance().m_autoAccelLimiter = new SlewRateLimiter(Units.feetToMeters(val),-1E9,DriveTrain.getInstance().m_autoSpeed)));
      public static double kAutoMaxRotSpeed = Units.degreesToRadians(NTDouble.create(130,"DriveConstants/kAuto/MaxRotSpeedDPS", val->kAutoMaxRotSpeed = Units.degreesToRadians(val)));
      public static double kAutoTurnToPoseDistance = Units.feetToMeters(NTDouble.create(20, "DriveConstants/kAuto/TurnToPoseDistanceFt", val->kAutoTurnToPoseDistance = Units.feetToMeters(val)));
      public static double kAutoSlowDist = Units.feetToMeters(NTDouble.create(5.0,"DriveConstants/kAuto/SlowDistFt",val->kAutoSlowDist=Units.feetToMeters(val)));
      public static double kAutoToleranceDistance = Units.inchesToMeters(NTDouble.create(1.0,"DriveConstants/kAuto/ToleranceDistanceIn",val->kAutoToleranceDistance=Units.inchesToMeters(val)));
      public static double kAutoToleranceMidPointDistance = Units.inchesToMeters(NTDouble.create(24, "DriveConstants/kAuto/ToleranceMidPointDistance", val->kAutoToleranceMidPointDistance = Units.inchesToMeters(val)));
      public static double kAutoToleranceAngle = Units.degreesToRadians(NTDouble.create(2,"DriveConstants/kAuto/ToleranceAngleDeg",val->kAutoToleranceAngle=Units.degreesToRadians(val)));
      public static double kAutoDriveLeadSeconds = NTDouble.create(0.0,"DriveConstants/kAuto/DriveLeadSeconds",val->kAutoDriveLeadSeconds=val);
      public static double kMinDriveSpeed = Units.feetToMeters(NTDouble.create(0.1,"DriveConstants/kMinDriveSpeedFPS",val->kMinDriveSpeed = Units.feetToMeters(val)));
      public static double kSpeedBoost = Units.feetToMeters(NTDouble.create(16, "DriveConstants/kSpeedBoostFPS", val->kSpeedBoost = Units.feetToMeters(val)));
      public static double kAttainableMaxSpeed = Units.feetToMeters(NTDouble.create(18, "DriveConstants/kAttainableMaxSpeedFPS", val->kAttainableMaxSpeed = Units.feetToMeters(val)));
      
      // how much to controller rotation to apply when in field centric mode
      public static double kRotationDirectControlRatio = NTDouble.create(0.7, "DriveConstants/kRotationDirectControlRatio", val->kRotationDirectControlRatio = val);

      public static double kPIDHeadingMaxRotSpeed = Units.degreesToRadians(NTDouble.create(120,"DriveConstants/kAuto/kPIDHeading/MaxRotSpeedDPS", val->kPIDHeadingMaxRotSpeed = Units.degreesToRadians(val)));
      public static double kPIDHeadingKp = NTDouble.create(2.0, "DriveConstants/kPIDHeading/Kp", (val)->{DriveTrain.getInstance().m_headingPID.setP(val);});
      public static double kPIDHeadingKi = NTDouble.create(0, "DriveConstants/kPIDHeading/Ki", (val)->{DriveTrain.getInstance().m_headingPID.setI(val);});
      public static double kPIDHeadingKd = NTDouble.create(0, "DriveConstants/kPIDHeading/Kd", (val)->{DriveTrain.getInstance().m_headingPID.setD(val);}); 
      public static double kPIDHeadingIntegratorRange = 0.1;

      public static double kGetItOffMeRotationSpeed = Units.degreesToRadians(NTDouble.create(360, "DriveConstants/kGetItOffMeRotSpeed",val -> kGetItOffMeRotationSpeed=Units.degreesToRadians(val)));

      public static double kStoppedRatio = NTDouble.create(0.001, "DriveConstants/kStoppedRatio", val->kStoppedRatio = val);

      public static double kAutoCrossTrackKp = NTDouble.create(3.8, "DriveConstants/kAutoCrossTrackKp", (val)->kAutoCrossTrackKp = val);
      public static double kAutoCrossTrackMax = Units.feetToMeters(NTDouble.create(6.7, "DriveConstants/kAutoCrossTrackMax", (val)->kAutoCrossTrackMax = Units.feetToMeters(val)));
    }

    public static class kSwerveModule {

      // CAN Bus ID Assignments
      public static CANID_t kCANID_FrontLeft = new CANID_t(2, 1, 2);
      public static CANID_t kCANID_FrontRight = new CANID_t(3, 10, 9);
      public static CANID_t kCANID_RearLeft = new CANID_t(0, 6, 5);
      public static CANID_t kCANID_RearRight = new CANID_t(1, 14, 13);

      public static MyMotorType kDriveType = MyMotorType.KRAKEN;
      public static MyMotorType kSteerType = MyMotorType.KRAKEN;
      public static double kCalibrationFrontLeft = (kDriveType == MyMotorType.KRAKEN ? 2487.0 : 1502.0);
      public static double kCalibrationFrontRight = (kDriveType == MyMotorType.KRAKEN ? 1104.0 : 2455.0);
      public static double kCalibrationRearLeft = (kDriveType == MyMotorType.KRAKEN ? 823.0 : 3634.0);
      public static double kCalibrationRearRight = (kDriveType == MyMotorType.KRAKEN ? 3949.0 : 2011.0);

      // Maximum Current Limits
      public static int kDrivingMotorCurrentLimit = NTInteger.create(40,"SwerveModule/kDrivingMotorCurrentLimit",val->DriveTrain.forEachSwerveModule((m)->{m.m_motorDrive.withStatorLimit(val);}));
      public static int kSteeringMotorCurrentLimit = NTInteger.create(5,"SwerveModule/kSteeringMotorCurrentLimit",val->DriveTrain.forEachSwerveModule((m)->{m.m_motorSteer.withStatorLimit(val);}));

      // Gear Ratios
      // Drive Characteristics
      public static double kWheelDiameterMeters = Units.inchesToMeters(4);

      // Drive Gear Ratios in order from motor to drive wheel
      public static double kDriveStage1Ratio = 14.0 / 50.0;
      public static double kDriveStage2RatioL1 = 25.0 / 19.0; // L1 MK4i Ratio Option Overall (8.14:1)
      public static double kDriveStage2RatioL2 = 27.0 / 17.0; // L2 MK4i Ratio Option Overall (6.75:1)
      public static double kDriveStage2RatioL3 = 28.0 / 16.0; // L3 MK4i Ratio Option Overall (6.12:1)
      public static double kDriveStage3Ratio = 15.0 / 45.0;
      public static double kDriveMotorGearReduction = 1/(kDriveStage1Ratio * (kDriveType == MyMotorType.NEO ? kDriveStage2RatioL1 : kDriveStage2RatioL2) * kDriveStage3Ratio);

      // Steering Gear Ratio
      public static double kSteerMotorGearReduction = 150.0/7.0;

      // Control Loop Gains - Drive
      public static double kDriveKp  = NTDouble.create(.0,"SwerveModule/kDriveKp",val->DriveTrain.forEachSwerveModule((m)->m.m_motorDrive.withKP(val,Motor.GainSlot.SPEED)));
      public static double kDrivePosKp  = NTDouble.create(0.5, "SwerveModule/kDrivePosKp",val->DriveTrain.forEachSwerveModule((m)->m.m_motorDrive.withKP(val, Motor.GainSlot.POSITION)));
      public static double kDriveKi  = NTDouble.create(.0, "SwerveModule/kDriveKi",val->DriveTrain.forEachSwerveModule((m)->m.m_motorDrive.withKI(val, Motor.GainSlot.SPEED)));
      public static double kDriveKd  = NTDouble.create(.0,"SwerveModule/kDriveKd",val->DriveTrain.forEachSwerveModule((m)->m.m_motorDrive.withKD(val, Motor.GainSlot.SPEED)));
      public static double kDriveKff = NTDouble.create(switch(kDriveType){case KRAKEN->.76; case NEO->0.096; default-> 0;},"SwerveModule/kDriveKff",val->DriveTrain.forEachSwerveModule((m)->m.m_motorDrive.withKFF(val, Motor.GainSlot.SPEED)));
      
      // Control Loop Gains - Steering
      public static double kSteerKp  = NTDouble.create(switch(kDriveType){case KRAKEN->3.14; case NEO->1.88; default-> 0;},"SwerveModule/kSteerKp",val->DriveTrain.forEachSwerveModule((m)->m.m_motorSteer.withKP(val, Motor.GainSlot.POSITION)));
      public static double kSteerKi  = NTDouble.create(0, "SwerveModule/kSteerKi",val->DriveTrain.forEachSwerveModule((m)->m.m_motorSteer.withKI(val, Motor.GainSlot.POSITION)));
      public static double kSteerKd  = NTDouble.create(0,"SwerveModule/kSteerKd",val->DriveTrain.forEachSwerveModule((m)->m.m_motorSteer.withKD(val, Motor.GainSlot.POSITION)));     
      public static double kSteerKff = 0;
    }

    // Kinematics Definition with Wheel Location Offsets
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kChassis.kWheelBase / 2, kChassis.kTrackWidth / 2),  // position Front Left
      new Translation2d(kChassis.kWheelBase / 2, -kChassis.kTrackWidth / 2),   // position Front Right
      new Translation2d(-kChassis.kWheelBase / 2, kChassis.kTrackWidth / 2), // position Rear Left
      new Translation2d(-kChassis.kWheelBase / 2, -kChassis.kTrackWidth / 2)   // position Rear Right
    );
  }
}
