// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.LongStream;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveModule;
//import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Vision;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;
import frc.utils.NTValues.NTInteger;
import frc.utils.Motor;
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

  public static class LEDConstants {
    public static double kMaxMotorTemp = (NTDouble.create(175.0, "LED/kMaxMotorTempF", (val)->kMaxMotorTemp = (val-32)/1.8) -32)/1.8;
    public static double kPoseResidualDist = Units.feetToMeters(NTDouble.create(2.0, "LED/kPoseResidualFeet", (val)->kPoseResidualDist = Units.feetToMeters(val)));
    public static double kPoseResidualRot = Units.degreesToRadians(NTDouble.create(30, "LED/kPoseResidualDeg", (val)->kPoseResidualRot = Units.degreesToRadians(val)));

    public static int kNumberLEDs = 96;
    public static int kLEDPortPWM = 9;

    public static class colors {
      public static int[] red = {255,0,0};
      public static int[] orange = {255,30,0};
      public static int[] yellow = {255,200,0};
      public static int[] green ={0,255,0};
      public static int[] blue = {0,0,255};
      public static int[] purple = {150,0,255};
      public static int[] pink = {255,0,150};
      public static int[] white = {255,255,255};
      public static int[] off = {0,0,0};
    }

    public static int[] kMotorTempColor = colors.red; 
    public static int[] kSetupFailColor = colors.yellow; 
    public static int[] kHealthyColor1 = colors.orange; 
    public static int[] kHealthyColor2 = colors.white; 
    public static int[] kActivityColor = colors.blue;
 
    public static long[] kAllLEDs = LongStream.range(0,96).toArray();
    
    public static long[] kIntakeLEDs = LongStream.concat(LongStream.range(0,10),LongStream.range(48,58)).toArray();
    public static long[] kSwerveLEDs = LongStream.concat(LongStream.range(10,21),LongStream.range(58,69)).toArray();
    public static long[] kGyroLEDs = LongStream.concat(LongStream.range(21,32),LongStream.range(69,80)).toArray();
    public static long[] kPhotonVisionLEDs = LongStream.concat(LongStream.range(32,40),LongStream.range(88,96)).toArray();
    public static long[] kPoseEstimatorLEDs = LongStream.concat(LongStream.range(40,48),LongStream.range(80,88)).toArray();

    public static double kUpdateTime = NTDouble.create(0.100, "LED/kUpdateTime", (val)->kUpdateTime=val);
    public static double kBrightness = NTDouble.create(0.1, "LED/kBrightness", (val)->kBrightness=val);
    public static double kFlashTime = NTDouble.create(0.5, "LED/kFlashTime", (val)->kFlashTime=val);
  
  }

  public static class OperatorConstants {
    // Controller Input Settings
    public static final int kDriverControllerPort = 0;
    public static final int kAccessoryControllerPort = 1;
    public static boolean kDriverControllerIsFlightStick = NTBoolean.create(true, "Operator/DriverControllerIsFlightStick",val->kDriverControllerIsFlightStick=val);

    // Default Settings
    public static boolean kFieldCentricDriving = true;
    public static boolean kDefaultDriveAccelLimiter = true;
    public static double kControllerActiveThreshold = 0.333;
  }

  public static class GyroConstants{
    public static double kVisionCorrectionMaxRate = Units.degreesToRadians(NTDouble.create(40,"Gyro/kVisionCorrectionMaxRate",val->kVisionCorrectionMaxRate=Units.degreesToRadians(val)));
    public static int kCANID_Pigeon = 50;
  }

  public static class PoseConstants{
    public static double kVisionWeightRotDecay = NTDouble.create(0.99, "Pose/kVisionWeightRotDecay", val->kVisionWeightRotDecay=val);
    public static double kVisionWeightRot = NTDouble.create(0.05, "Pose/kVisionWeightTheta", val->kVisionWeightRot=val);
    public static double kVisionWeightPos = NTDouble.create(0.2, "Pose/kVisionWeightPos", val->kVisionWeightPos=val);
    public static double kGyroWeight = NTDouble.create(0.10, "Pose/kGyroWeight",val->kGyroWeight=val);
    public static double kDriveSlip = NTDouble.create(1.0, "Pose/kDriveSlip",val->kDriveSlip=val);;
  }

  public static class VisionConstants{
    public static double kExtraLatencyMillis = NTDouble.create(120, "Vision/kExtraLatencyMillis", val->kExtraLatencyMillis=val);
    public static double kMaxAmbiguity = NTDouble.create(0.2, "Vision/kMaxAmbiguity", val->kMaxAmbiguity=val);
    public static int    kPipelineIndex = NTInteger.create(0, "Vision/kPipelineIndex", val->Vision.cam1.setPipelineIndex(val));
    public static double kMinTargetArea = NTDouble.create(.05, "Vision/kMinTargetArea", val->kMinTargetArea=val);

    public static String kFieldLayout = Filesystem.getDeployDirectory()+"/2025-ReefScape.json";

    public static double kMaxLatencyCompensationMillis = 200; // ms

    public static double kPixyTimeout = 1; // second


    // Simulated Vision System
    public static class frontCamera{
      public static final String kCameraName = "FrontCam";

      public static double kCamYawLeft = Math.toRadians(NTDouble.create(0, "Vision/"+kCameraName+"/kCamYawLeft_deg", val->{kCamYawLeft=Math.toRadians(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamPitchUp = Math.toRadians(NTDouble.create(15, "Vision/"+kCameraName+"/kCamPitchUp_deg", val->{kCamPitchUp=Math.toRadians(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamDiagFOV = Math.toRadians(NTDouble.create(77.2, "Vision/"+kCameraName+"/kCamDiagFOV_deg", val->{kCamDiagFOV=Math.toRadians(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamForwardOffset = Units.inchesToMeters(NTDouble.create(-2, "Vision/"+kCameraName+"/kCamForwardOffset_in", val->{kCamForwardOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamLeftOffset = Units.inchesToMeters(NTDouble.create(-11.5, "Vision/"+kCameraName+"/kCamLeftOffset_in", val->{kCamLeftOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorFront.setRobotToCameraTransform(kCameraToRobot().inverse());}));

      public static double kSimMaxLEDRange = 20; // meters
      public static int    kSimCamResolutionW = 1280; // pixels
      public static int    kSimCamResolutionH = 720; // pixels
      
      public static double kCamHeightOffGround = Units.inchesToMeters(18.375);
      public static Transform3d kCameraToRobot(){
        return new Transform3d(
          new Translation3d(
            -kCamForwardOffset,
            -kCamLeftOffset,
            -kCamHeightOffGround
          ),
          new Rotation3d(0.0, kCamPitchUp, kCamYawLeft)
        );
      }
    }

    public static class rearCamera{
      public static final String kCameraName = "RearCam";
    
    
      public static double kCamYawLeft = Math.toRadians(NTDouble.create(180, "Vision/"+kCameraName+"/kCamYawLeft_deg", val->{kCamYawLeft=Math.toRadians(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamPitchUp = Math.toRadians(NTDouble.create(-35, "Vision/"+kCameraName+"/kCamPitchUp_deg", val->{kCamPitchUp=Math.toRadians(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamDiagFOV = Math.toRadians(NTDouble.create(76.06, "Vision/"+kCameraName+"/kCamDiagFOV_deg", val->{kCamDiagFOV=Math.toRadians(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamForwardOffset = Units.inchesToMeters(NTDouble.create(-6, "Vision/"+kCameraName+"/kCamForwardOffset_in", val->{kCamForwardOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));
      public static double kCamLeftOffset = Units.inchesToMeters(NTDouble.create(-0.25, "Vision/"+kCameraName+"/kCamLeftOffset_in", val->{kCamLeftOffset=Units.inchesToMeters(val); Vision.photonPoseEstimatorRear.setRobotToCameraTransform(kCameraToRobot().inverse());}));

      public static double kSimMaxLEDRange = 20; // meters
      public static int    kSimCamResolutionW = 1280; // pixels
      public static int    kSimCamResolutionH = 720; // pixels
      
      public static double kCamHeightOffGround = Units.inchesToMeters(7);
      public static Transform3d kCameraToRobot(){
        return new Transform3d(
          new Translation3d(
            -kCamForwardOffset,
            -kCamLeftOffset,
            -kCamHeightOffGround
          ),
          new Rotation3d(0.0, kCamPitchUp, kCamYawLeft)
        );
      }
    }
  }

  // Chassis configuration
  public static class kChassis {
    public static final double kTrackWidth = Units.inchesToMeters(20.75);  // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(22.75);       // Distance between front and back wheels
  }

  /* public static class IntakeConstants {
    // Velocity Control PID Gains
    public static double kIntakeRollerKp  = NTDouble.create(0.00002,"Intake/kRollerKp",val->{ kIntakeRollerKp = val; IntakeShooter.getInstance().m_pidIntake.setP(val,0);});
    public static double kIntakeRollerKi  = NTDouble.create(0, "Intake/kRollerKi",val->{ kIntakeRollerKi = val; IntakeShooter.getInstance().m_pidIntake.setI(val,0);});
    public static double kIntakeRollerKd  = NTDouble.create(0,"Intake/kRollerKd",val->{ kIntakeRollerKd = val; IntakeShooter.getInstance().m_pidIntake.setD(val,0);});
    public static double kIntakeRollerKff = NTDouble.create(0.005,"Intake/kRollerKff",val ->{ kIntakeRollerKff = val; IntakeShooter.getInstance().m_pidIntake.setFF(val,0);});
    // Position Control PID Gains
    public static double kIntakeRollerPosKp  = NTDouble.create(8, "Intake/kRollerPosKp",val->{ kIntakeRollerPosKp = val; IntakeShooter.getInstance().m_pidIntake.setP(val,1);});
    public static double kIntakeRollerPosKi  = NTDouble.create(0, "Intake/kRollerPosKi",val->{ kIntakeRollerPosKi = val; IntakeShooter.getInstance().m_pidIntake.setI(val,1);});
    public static double kIntakeRollerPosKd  = NTDouble.create(0,"Intake/kRollerPosKd",val->{ kIntakeRollerPosKd = val; IntakeShooter.getInstance().m_pidIntake.setD(val,1);});
    public static double kIntakeRollerPosKff = NTDouble.create(0,"Intake/kRollerPosKff",val ->{ kIntakeRollerPosKff = val; IntakeShooter.getInstance().m_pidIntake.setFF(val,1);});
      
    public static double kRollerDiameter = Units.inchesToMeters(1.375);
    public static double kEncoderVelocityFactor = 1/5.0;
    public static double kEncoderPositionFactor = 1/5.0*(Math.PI*kRollerDiameter);

    public static double kMaxSpeed = NTDouble.create(2000, "Intake/kMaxSpeed", val->kMaxSpeed=val);
    public static int kCANID_Intake = 8;
    public static int kBeamBreakPin = 9;
    public static double kBeamBreakDebounceSec = NTDouble.create(0.010, "Intake/kBeamBreakDebounceSec", val->{kBeamBreakDebounceSec=val; IntakeShooter.getInstance().m_beamDebouncer = new Debouncer(val, Debouncer.DebounceType.kBoth);});
    public static boolean kIntakeIsInverted = false;
    public static double kRetractDistance = NTDouble.create(10, "Intake/kRetractDistance", val->{kRetractDistance=val;});
  } */

  /* public static class ShooterConstants {
    public static double kMaxSpeed = NTDouble.create(3000,"Shooter/kMaxSpeed",val->kMaxSpeed = val);
    public static double kMaxSpeedBack = NTDouble.create(1000,"Shooter/Back/kMaxBackSpeed",val->kMaxSpeed = val);
    public static double kAtMaxSpeedPercent = 0.8;
        
    public static double kShooterVelocityFactor = 1;

    public static boolean kShooterIsInverted = false;

    public static int kCANID_Shooter = 7;

    public static double kShooterKp  = NTDouble.create(0.0004,"Shooter/kShooterKp",val->{ kShooterKp = val; IntakeShooter.getInstance().m_pidShooter.setP(val); IntakeShooter.getInstance().m_pidShooter.setP(val);});
    public static double kShooterKi  = NTDouble.create(0.000008, "Shooter/kShooterKi",val->{ kShooterKi = val; IntakeShooter.getInstance().m_pidShooter.setI(val); IntakeShooter.getInstance().m_pidShooter.setI(val);});
    public static double kShooterKd  = NTDouble.create(0.0,"Shooter/kShooterKd",val->{ kShooterKd = val; IntakeShooter.getInstance().m_pidShooter.setD(val); IntakeShooter.getInstance().m_pidShooter.setD(val);});
    public static double kShooterKff = NTDouble.create(0.0002,"Shooter/kShooterKff",val ->{ kShooterKff = val; IntakeShooter.getInstance().m_pidShooter.setFF(val); IntakeShooter.getInstance().m_pidShooter.setFF(val);});

    // Angle constants

    public static int kCANID_AngleLeft = 4;
    public static int kCANID_AngleRight = 12;
    public static int kAnalogInputpin = 4;

    public static double kAngleKp  = NTDouble.create(0.9,"Shooter/kAngleKp",val->{ kAngleKp = val; IntakeShooter.getInstance().m_pidAngle.setP(val); IntakeShooter.getInstance().m_pidAngleRight.setP(val);});
    public static double kAngleKi  = NTDouble.create(0.0, "Shooter/kAngleKi",val->{ kAngleKi = val; IntakeShooter.getInstance().m_pidAngle.setI(val); IntakeShooter.getInstance().m_pidAngleRight.setI(val);});
    public static double kAngleKd  = NTDouble.create(60,"Shooter/kAngleKd",val->{ kAngleKd = val; IntakeShooter.getInstance().m_pidAngle.setD(val); IntakeShooter.getInstance().m_pidAngleRight.setD(val);});
    public static double kAngleKff = NTDouble.create(0,"Shooter/kAngleKff",val ->{ kAngleKff = val; IntakeShooter.getInstance().m_pidAngle.setFF(val); IntakeShooter.getInstance().m_pidAngleRight.setFF(val);});

    public static double kAngleEncoderPositionFactor = (2 * Math.PI/70);

    // Calibration map for intake shooter

    public static double [] kShooterAngleCalibrationX = new double[]{638, 1137,  1372, 1654,  1959, 2172, 2404, 2462, 2482}; //analog values
    public static double [] kShooterAngleCalibrationY = new double[]{131, 105,   90,   70,    45,   25,    0, -6.0, -8.0}; // degrees

    // perform unit conversion on kShooterAngleCalibration degrees to radians
    static {for (int i=0; i<kShooterAngleCalibrationY.length; i++){ kShooterAngleCalibrationY[i] = Units.degreesToRadians(kShooterAngleCalibrationY[i]); }};

    public static int kAngleCurrentLimit = 70;
    public static double kAngleMax = Units.degreesToRadians(NTDouble.create(105, "Shooter/kAngleMax", val -> kAngleMax = Units.degreesToRadians(val)));
    public static double kAngleMin = Units.degreesToRadians(NTDouble.create(-4, "Shooter/kAngleMin", val -> kAngleMin = Units.degreesToRadians(val)));
    public static double kManualAngleSpeed = Units.degreesToRadians(NTDouble.create(90, "Shooter/kManualAngleSpeed", val -> kManualAngleSpeed = Units.degreesToRadians(val)));
    public static double kAngleTolerance = Units.degreesToRadians(NTDouble.create(15, "Shooter/kAngleTolerance", val -> kAngleTolerance = Units.degreesToRadians(val)));

    public static InterpolatingDoubleTreeMap kAngleDistMap = new InterpolatingDoubleTreeMap();


    static {kAngleDistMap.put(Units.feetToMeters(3.15), Units.degreesToRadians(14.0)); } // 18
    static {kAngleDistMap.put(Units.feetToMeters(6.0), Units.degreesToRadians(32.0)); }
    static {kAngleDistMap.put(Units.feetToMeters(8.0), Units.degreesToRadians(33.4)); }
    static {kAngleDistMap.put(Units.feetToMeters(10), Units.degreesToRadians(33.32)); }

    public static double kAngleDistAMin= Units.degreesToRadians(NTDouble.create(12, "Shooter/kAngleDistAMin", val -> kAngleDistAMin = Units.degreesToRadians(val)));
    public static double kAngleDistAMax= Units.degreesToRadians(NTDouble.create(28, "Shooter/kAngleDistAMax", val -> kAngleDistAMax = Units.degreesToRadians(val)));
    public static double kAngleDistDMin= Units.feetToMeters(NTDouble.create(4.4, "Shooter/kAngleDistDMin", val -> kAngleDistDMin = Units.feetToMeters(val)));
    public static double kAngleDistDMax= Units.feetToMeters(NTDouble.create(9.4, "Shooter/kAngleDistDMax", val -> kAngleDistDMax = Units.feetToMeters(val)));
    public static double kSpeakerAimDistance = Units.feetToMeters(NTDouble.create(-0.25, "Shooter/kSpeakerAimDistance", val -> kAngleDistDMax = Units.feetToMeters(val)));
    public static double kAngleDistDerK = NTDouble.create(0.15, "Shooter/kAngleDistDerK", val -> kAngleDistDerK = val);
    public static double kAngleDistGain = Units.degreesToRadians(NTDouble.create(-2,"Shooter/kAngleDistGainDegPerFPS", val -> kAngleDistGain = Units.degreesToRadians(val)/Units.feetToMeters(1)))/Units.feetToMeters(1);
    public static double kVelocityMaxAdjust = Units.degreesToRadians(NTDouble.create(10,"Shooter/kVelocityMaxAdjustDeg", val -> kVelocityMaxAdjust = Units.degreesToRadians(val)));

    public static class kAnglePreset {
      public static double Amp = Units.degreesToRadians(NTDouble.create(102,"Intake/kAnglePreset/Amp", val->Amp=Units.degreesToRadians(val)));
      public static double Speaker = Units.degreesToRadians(NTDouble.create(19,"Intake/kAnglePreset/Speaker", val->Speaker=Units.degreesToRadians(val)));
      public static double Ground = Units.degreesToRadians(NTDouble.create(-3,"Intake/kAnglePreset/Ground", val->Ground=Units.degreesToRadians(val)));
      public static double Up = Units.degreesToRadians(NTDouble.create(90,"Intake/kAnglePreset/Up", val->Up=Units.degreesToRadians(val)));
    }
  } */

  /* public static class ClimberConstants {
    public static int kCANIDLeft = 3;
    public static int kCANIDRight = 11;
    public static double kPIDKp  = NTDouble.create(30,"Climber/PID/Kp",val->{ kPIDKp = val; Climber.right().m_pidClimber.setP(val); Climber.left().m_pidClimber.setP(val);});
    public static double kPIDKi  = NTDouble.create(0,"Climber/PID/Ki",val->{ kPIDKi = val; Climber.right().m_pidClimber.setI(val); Climber.left().m_pidClimber.setI(val);});
    public static double kPIDKd  = NTDouble.create(0,"Climber/PID/Kd",val->{ kPIDKd = val; Climber.right().m_pidClimber.setD(val); Climber.left().m_pidClimber.setD(val);});
    public static double kPIDKff  = NTDouble.create(0,"Climber/PID/Kff",val->{ kPIDKff = val; Climber.right().m_pidClimber.setFF(val); Climber.left().m_pidClimber.setFF(val);});
   
    public static int kCurrentInit = (int)NTDouble.create(3,"Climber/kCurrentInit",val->kCurrentInit=(int)val);
    public static int kCurrentLimit = (int)NTDouble.create(50,"Climber/kCurrentLimit",val->{kCurrentLimit=(int)val; Climber.right().m_climber.setSmartCurrentLimit((int)val); Climber.left().m_climber.setSmartCurrentLimit((int)val);});
    
    public static double kMaxHeight = Units.inchesToMeters(NTDouble.create(41,"Climber/kMaxHeightInch",val->kMaxHeight=Units.inchesToMeters(val)));
    public static double kMinHeight = Units.inchesToMeters(NTDouble.create(21.5,"Climber/kMinHeightInch",val->kMinHeight=Units.inchesToMeters(val)));
    public static double kSpoolDiameter = Units.inchesToMeters(1);
    //public static double kGearRatio = 120;
    public static double kGearRatio = 60;
    public static double kEncoderVelocityFactor = Math.PI*kSpoolDiameter/kGearRatio;
    public static double kEncoderPositionFactor = Math.PI*kSpoolDiameter/kGearRatio;
    public static double kInitRetractMargin = Units.inchesToMeters(1);
    public static double kInitTime = NTDouble.create(0.25,"Climber/kInitTime",val->kInitTime=val);
    public static double kInitMaxEffort = NTDouble.create(.1,"Climber/kInitMaxEffort",val->kInitMaxEffort=val);
    public static double kInitCurrentLimit = NTDouble.create(3,"Climber/kInitCurrentLimit",val->kInitCurrentLimit=val);

    public static double kMaxSpeed = Units.inchesToMeters(NTDouble.create(6, "Climber/kMaxSpeed", val -> kMaxSpeed = Units.inchesToMeters(val)));
  } */

  public static class AutoConstants {

    public static double kSpeakerNearDistance = Units.feetToMeters(NTDouble.create(6.5, "Autonomous/kSpeakerNearDistance", val -> kSpeakerNearDistance = Units.feetToMeters(val)));
    public static double kSpeakerDistance = Units.feetToMeters(NTDouble.create(6, "Autonomous/kSpeakerDistance", val -> kSpeakerDistance = Units.feetToMeters(val)));
    public static double kSpeakerMaxAngle = Units.degreesToRadians(NTDouble.create(38, "Autonomous/kSpeakerMaxAngle", val -> kSpeakerMaxAngle = Units.degreesToRadians(val)));
    public static double kAmpDistance = Units.feetToMeters(NTDouble.create(0.4, "Autonomous/kAmpDistance", val -> kAmpDistance = Units.feetToMeters(val)));
    public static double kAmpDistanceInitial = Units.feetToMeters(NTDouble.create(2.5, "Autonomous/kAmpDistanceInitial", val -> kAmpDistanceInitial = Units.feetToMeters(val)));
    public static double kSourceLDistance = Units.feetToMeters(NTDouble.create(4, "Autonomous/kSourceLDistance", val -> kSourceLDistance = Units.feetToMeters(val)));
    public static double kSourceRDistance = Units.feetToMeters(NTDouble.create(4, "Autonomous/kSourceRDistance", val -> kSourceRDistance = Units.feetToMeters(val)));
    public static double kNoteNearDistance = Units.feetToMeters(NTDouble.create(5, "Autonomous/kNoteNearDistance", val -> kNoteNearDistance = Units.feetToMeters(val)));
    public static double kNoteDistance = Units.feetToMeters(NTDouble.create(1.9, "Autonomous/kNoteDistance", val -> kNoteDistance = Units.feetToMeters(val)));
    public static double kaimNoteGain = NTDouble.create(0.05,"Auto/kAimNoteGain",val->kaimNoteGain=val);
    
    public static double kFieldLength = 16.451;
    public static double kFieldWidth = 8.211;
    public static double kNoteSpacing = Units.feetToMeters(5.5);
    public static ArrayList<Pose2d> kCenterNotes = new ArrayList<>(Arrays.asList(
      new Pose2d(new Translation2d(kFieldLength/2.0, kFieldWidth/2.0 - (2 * kNoteSpacing)), null),
      new Pose2d(new Translation2d(kFieldLength/2.0, kFieldWidth/2.0 - (1 * kNoteSpacing)), null),
      new Pose2d(new Translation2d(kFieldLength/2.0, kFieldWidth/2.0 - (0 * kNoteSpacing)), null),
      new Pose2d(new Translation2d(kFieldLength/2.0, kFieldWidth/2.0 + (1 * kNoteSpacing)), null),
      new Pose2d(new Translation2d(kFieldLength/2.0, kFieldWidth/2.0 + (2 * kNoteSpacing)), null))
    );

    
    private static ArrayList<Pose2d> kBlueNotes = new ArrayList<>(Arrays.asList(
      new Pose2d(new Translation2d(2.85, 4.125), null),
      new Pose2d(new Translation2d(2.85, 5.55), null),
      new Pose2d(new Translation2d(2.85, 7.00), null)));

    public static ArrayList<Pose2d> kAllianceNotes = new ArrayList<>(kBlueNotes);

    public static void calcAllianceNotes(boolean isBlue){
      if (isBlue){
        kAllianceNotes.clear();
        kAllianceNotes.addAll(kBlueNotes);
      } else {
        // Flip across halffield for red
        kAllianceNotes.clear();
        kAllianceNotes.addAll(kBlueNotes.stream().map(f->new Pose2d(new Translation2d(kFieldLength/2.0+(kFieldLength/2.0-f.getX()),f.getY()),null)).collect(Collectors.toList()));
      }
    }
  }

  public static class kDriveTrain {
    
    public static class DriveConstants {
      public static double kLinearAccelerationTau = NTDouble.create(0.01, "DriveConstants/kLinearAccelerationTau", val->DriveTrain.forEachSwerveModule((m)->m.m_magLimiter.tau=val));

      public static Integer kMaxSpeedDefault = 0; // 0 indexed selector for arrays below

      public static double [] kMaxSpeedMetersPerSecond = { // m/s
        Units.feetToMeters(7),
        Units.feetToMeters(10),
        Units.feetToMeters(18)
      };

      public static double [] kMaxRotationVelocity = { // rad/s
        Units.degreesToRadians(75), 
        Units.degreesToRadians(75),
        Units.degreesToRadians(75)
      };
      public static double kMaxSpeed = Units.feetToMeters(NTDouble.create(4, "DriveConstants/MaxSpeed",val -> DriveTrain.getInstance().setMaxSpeed(Units.feetToMeters(val))));
      public static boolean kAutoTurnToBestTag = NTBoolean.create(false,"DriveConstants/kAuto/TurnToBestTag",val->kAutoTurnToBestTag=val);
      public static double kAutoMaxSpeed = Units.feetToMeters(NTDouble.create(8,"DriveConstants/kAuto/MaxSpeedFPS", val->kAutoMaxSpeed = Units.feetToMeters(val)));
      public static double kAutoMinSpeed = Units.feetToMeters(NTDouble.create(1.25,"DriveConstants/kAuto/MinSpeedFPS", val->kAutoMinSpeed = Units.feetToMeters(val)));
      public static double kAutoAccelLimiter = Units.feetToMeters(NTDouble.create(8,"DriveConstants/kAuto/AccelLimiterFPS2", val->DriveTrain.getInstance().m_autoAccelLimiter = new SlewRateLimiter(Units.feetToMeters(val),-1E9,DriveTrain.getInstance().m_autoSpeed)));
      public static double kAutoMaxRotSpeed = Units.degreesToRadians(NTDouble.create(120,"DriveConstants/kAuto/MaxRotSpeedDPS", val->kAutoMaxRotSpeed = Units.degreesToRadians(val)));
      public static double kAutoTurnToPoseDistance = Units.feetToMeters(NTDouble.create(10, "DriveConstants/kAuto/TurnToPoseDistanceFt", val->kAutoTurnToPoseDistance = Units.feetToMeters(val)));
      public static double kAutoSlowDist = Units.feetToMeters(NTDouble.create(4,"DriveConstants/kAuto/SlowDistFt",val->kAutoSlowDist=Units.feetToMeters(val)));
      public static double kAutoToleranceDistance = Units.feetToMeters(NTDouble.create(0.15,"DriveConstants/kAuto/ToleranceDistance",val->kAutoToleranceDistance=Units.feetToMeters(val)));
      public static double kAutoToleranceAngle = Units.degreesToRadians(NTDouble.create(2,"DriveConstants/kAuto/ToleranceAngle",val->kAutoToleranceAngle=Units.degreesToRadians(val)));

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

      public static double kStoppedRatio = NTDouble.create(0.03, "DriveConstants/kStoppedRatio", val->kStoppedRatio = val);

      public static double kMidPointAccuracyFactor = NTDouble.create(3, "DriveConstants/kMidPointAccuracyFactor", val->kMidPointAccuracyFactor = val);;

    }

    public static class kSwerveModule {

      // CAN Bus ID Assignments
      public static CANID_t kCANID_FrontLeft = new CANID_t(2, 1, 2);
      public static CANID_t kCANID_FrontRight = new CANID_t(3, 10, 9);
      public static CANID_t kCANID_RearLeft = new CANID_t(0, 6, 5);
      public static CANID_t kCANID_RearRight = new CANID_t(1, 14, 13);

      
      public static double kCalibrationFrontLeft = 454.0;
      public static double kCalibrationFrontRight = 2455.0;
      public static double kCalibrationRearLeft = 3634.0;
      public static double kCalibrationRearRight = 2011.0;

      // Maximum Current Limits
      public static double kDrivingMotorCurrentLimit = NTDouble.create(40,"SwerveModule/kDrivingMotorCurrentLimit",val->DriveTrain.forEachSwerveModule((m)->{m.m_motorDrive.smartCurrentLimit((int)val);}));
      public static double kSteeringMotorCurrentLimit = NTDouble.create(5,"SwerveModule/kSteeringMotorCurrentLimit",val->DriveTrain.forEachSwerveModule((m)->{m.m_motorSteer.smartCurrentLimit((int)val);}));

      // Gear Ratios
      // Drive Characteristics
      public static double kMaxFreeRunSpeed = 5676; // Specific to Rev Robotics NEO Brushless Motor
    
      public static double kWheelDiameterMeters = Units.inchesToMeters(4);

      // Drive Gear Ratios in order from motor to drive wheel
      public static double kDriveStage1Ratio = 14.0 / 50.0;
      //public static double kDriveStage2Ratio = 25.0 / 19.0; // L1 MK4i Ratio Option Overall (8.14:1)
      public static double kDriveStage2Ratio = 27.0 / 17.0; // L2 MK4i Ratio Option Overall (6.75:1)
   // public static double kDriveStage2Ratio = 28.0 / 16.0; // L3 MK4i Ratio Option Overall (6.12:1)
      public static double kDriveStage3Ratio = 15.0 / 45.0;
      public static double kDriveMotorGearReduction = 1/(kDriveStage1Ratio * kDriveStage2Ratio * kDriveStage3Ratio);

      // Steering Gear Ratio
      public static double kSteerMotorGearReduction = 150.0/7.0;
      //public static double kSteerMotorStage1Ratio = 14.0/50.0;
      //public static double kSteerMotorStage2Ratio = 14.0/60.0;
      //public static double kSteerMotorGearReduction = kSteerMotorStage1Ratio*kSteerMotorStage2Ratio;

      // Encoder Scaling Factors
      public static double kDriveEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDriveMotorGearReduction; //(kWheelDiameterMeters * Math.PI) / kDriveMotorGearReduction;
      public static double kDriveEncoderVelocityFactor = (kWheelDiameterMeters * Math.PI) / kDriveMotorGearReduction / 60.0;
      public static double kSteerEncoderPositionFactor = (2.0 * Math.PI) / kSteerMotorGearReduction;
      public static double kSteerEncoderVelocityFactor = (2.0 * Math.PI) / kSteerMotorGearReduction / 60.0;

      // Control Loop Gains - Drive

      private static void updateDrivepidf(SwerveModule module, Double p, Double i, Double d, Double ff) {
        module.m_motorDrive.pidf(
          p != null ? p : kDriveKp,
          i != null ? i : kDriveKi,
          d != null ? d : kDriveKd,
          ff != null ? ff : kDriveKff
        );
      }

      public static double kDriveKp  = NTDouble.create(.2,"SwerveModule/kDriveKp",val->DriveTrain.forEachSwerveModule((m)-> updateDrivepidf(m, val, null, null, null)));
      public static double kDriveKi  = NTDouble.create(.001, "SwerveModule/kDriveKi",val->DriveTrain.forEachSwerveModule((m)-> updateDrivepidf(m, null, val, null, null)));
      public static double kDriveKd  = NTDouble.create(0,"SwerveModule/kDriveKd",val->DriveTrain.forEachSwerveModule((m)-> updateDrivepidf(m, null, null, val, null)));
      public static double kDriveKff = NTDouble.create(0.3,"SwerveModule/kDriveKff",val->DriveTrain.forEachSwerveModule((m)-> updateDrivepidf(m, null, null, null, val)));
      
      // Control Loop Gains - Steering

      private static void updateSteerpidf(SwerveModule module, Double p, Double i, Double d, Double ff) {
        module.m_motorSteer.pidf(
          p != null ? p : kSteerKp,
          i != null ? i : kSteerKi,
          d != null ? d : kSteerKd,
          ff != null ? ff : kSteerKff
        );
      }

      public static double kSteerKp  = NTDouble.create(0.3,"SwerveModule/kSteerKp",val->DriveTrain.forEachSwerveModule((m)-> updateSteerpidf(m, val, null, null, null)));
      public static double kSteerKi  = NTDouble.create(0, "SwerveModule/kSteerKi",val->DriveTrain.forEachSwerveModule((m)-> updateSteerpidf(m, null, val, null, null)));
      public static double kSteerKd  = NTDouble.create(0,"SwerveModule/kSteerKd",val->DriveTrain.forEachSwerveModule((m)-> updateSteerpidf(m, null, null, val, null)));      
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
