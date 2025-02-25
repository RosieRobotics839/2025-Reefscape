// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.LongStream;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Funnel;
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

  public static class GameConstants {
    public enum ScoreLevel{
      TROUGH, LEVEL2, LEVEL3, LEVEL4
    }
  }
  
  public static class ArmConstants {
    public static double kLimitUnderDZ = Units.inchesToMeters(NTDouble.create(9.0,"GameConstants/DZ/kLimUnderDZ",(val)->kLimitUnderDZ=Units.inchesToMeters(val)));; // The Limit for how high the Elevator can go under the danger zone (dz / 9 - 21 inches)
    public static double kLimitAboveDZ = Units.inchesToMeters(NTDouble.create(21.0,"GameConstants/DZ/kLimAboveDZ",(val)->kLimitAboveDZ=Units.inchesToMeters(val)));; // The Limit for how high the Elevator can go above the danger zone (dz / 9 - 21 inches)
    public static double kAngleMaxDZ = Units.degreesToRadians(NTDouble.create(67.0,"GameConstants/DZ/kAngleMax",(val)->kAngleMaxDZ=Units.degreesToRadians(val))); // The Max Angle in degrees for the Arm while it is in the Danger Zone (dz)
  
    public static MyMotorType kMotorType = MyMotorType.KRAKEN;
    public static int kCANID = 4;
    public static int kDigitalInputID = 1;
    
    public static Motor.Gains kGainPosition = MotorDefaults.Kraken.kGainPosition;

    // Creates Max and Min values for Arm Software Hardstop
    public static double kAngleMax = (NTDouble.create(89, "Arm/kAngleMax", (val)->kAngleMax = (val)));
    public static double kAngleMin = (NTDouble.create(-3, "Arm/kAngleMin", (val)->kAngleMin = (val)));

    // TODO: Change once angles are settled on.
    public static double kTargetAngleTrough = 0;
    public static double kTargetAngleLevelMiddle = 0;
    public static double kTargetAngleLevel4 = 0;

    // TODO: Change values of Calibration Maps.
    public static double [] kCalibrationX = new double[]{ 0.333, 0.598}; //analog values
    public static double [] kCalibrationY = new double[]{Units.degreesToRadians(-2), Units.degreesToRadians(90)};

    public static double kArmMotorCurrentLimit = (NTDouble.create(5, "Arm/kCurrentLimit", (val) ->Arm.getInstance().m_motor.withStatorLimit(val)));

    public static double kAngleTolerance = Units.degreesToRadians(NTDouble.create(0.5, "Arm/kArmAngleTolerance", val -> kAngleTolerance = Units.degreesToRadians(val)));
    public static double kArmGearRatio = 40;
    public static double kMaxSpeed = 0.5;
  }

  public static class EffectorConstants {
    public static MyMotorType kMotorType = MyMotorType.NEO;
    public static int kCANID = 7; //Change later
    public static double kEffectorStage1Ratio = 5.0 / 1.0; //Starting with a 5:1 Gear Ratio.
    public static double kEffectorMotorGearReduction = kEffectorStage1Ratio;
    public static double kEffectorEncoderPositionFactor = (2.0 * Math.PI) / kEffectorMotorGearReduction;
    public static double kSpeed = 0; //Change once we can test
    public static double kAlgaeMotorRevolutions = 5;

    public static double kMaxSpeed = 1; // Mechanism rotations per second
    public static int kBeamBreakPin = 9;
    public static double kBeamBreakDebounceSec = NTDouble.create(0.010, "Intake/kBeamBreakDebounceSec", val->{kBeamBreakDebounceSec=val; EndEffector.getInstance().m_beamDebouncer = new Debouncer(val, Debouncer.DebounceType.kBoth);});
    public static boolean kIntakeIsInverted = false;
    public static double kRetractDistance = NTDouble.create(10, "Intake/kRetractDistance", val->{kRetractDistance=val;});

    public static double kMotorCurrentLimit = (NTDouble.create(5, "Effector/kCurrentLimit", (val) ->EndEffector.getInstance().m_motor.withStatorLimit(val)));

    // Position control gains
    public static Motor.Gains kGainPosition = new Motor.Gains(0.3, 0.002, 0, 0);
    public static Motor.Gains kGainVelocity = new Motor.Gains(0, 0, 0, 0);

    public static double kGearRatio = 5;
  }
  public static class ElevatorConstants {

    public static MyMotorType kMotorType = MyMotorType.KRAKEN;
    public static int kEleLeftCANID = 11;
    public static int kEleRightCANID = 12;

    public static double kLimitUnderDZ = Units.inchesToMeters(NTDouble.create(9.0,"GameConstants/DZ/kLimUnderDZ",(val)->kLimitUnderDZ=Units.inchesToMeters(val)));; // The Limit for how high the Elevator can go under the danger zone (dz / 9 - 21 inches)
    public static double kLimitAboveDZ = Units.inchesToMeters(NTDouble.create(21.0,"GameConstants/DZ/kLimAboveDZ",(val)->kLimitAboveDZ=Units.inchesToMeters(val)));; // The Limit for how high the Elevator can go above the danger zone (dz / 9 - 21 inches)

    public static int klimitSwitchChannel;

    public static double kLeftElevatorMotorCurrentLimit = (NTDouble.create(30, "Elevator/kLeftCurrentLimit", (val) ->Elevator.getInstance().m_EleMotorLeft.withStatorLimit(val)));
    public static double kRightElevatorMotorCurrentLimit = (NTDouble.create(30, "Elevator/kRightCurrentLimit", (val) ->Elevator.getInstance().m_EleMotorRight.withStatorLimit(val)));

    // Values in inches that the elevator should be raised from the bottom to score different heights of coral and algae
    // These need to be tested and adjusted
    public static double kMaxHeightInch = Units.inchesToMeters(NTDouble.create(27.5, "Elevator/MaxHeightInch", (val)->kMaxHeightInch = Units.inchesToMeters(val)));; // 16.6 Rotations of the Axle
    public static double kHeight3Inch = Units.inchesToMeters(NTDouble.create(16, "Elevator/Height3Inch", (val)->kHeight3Inch = Units.inchesToMeters(val)));
    public static double kHeight2Inch = Units.inchesToMeters(NTDouble.create(8, "Elevator/Height2Inch", (val)->kHeight2Inch = Units.inchesToMeters(val)));
    public static double kHeight1Inch = Units.inchesToMeters(NTDouble.create(1, "Elevator/Height1Inch", (val)->kHeight1Inch = Units.inchesToMeters(val)));
    public static double kMinHeightInch = Units.inchesToMeters(NTDouble.create(0, "Elevator/MinHeightInch", (val)->kMinHeightInch = Units.inchesToMeters(val)));

    public static double kElevatorTolerance = Units.inchesToMeters(0.5);
    public static double kMaxSpeedPositive = 10; // Rotations per second
    public static double kMaxSpeedNegative = -4; // Rotations per second
    public static double kElevatorGearRatio = 16;
    public static double kSprocketDiameter = Units.inchesToMeters(1.685);

    public static Motor.Gains kGainPosition = new Motor.Gains(19.2,0.625,0,0);
    
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
    public static double kCurrentLimit = 5;
    public static boolean kInverted = false;
    public static Boolean kIdleBrake = false;
  
    // If motor rotations per second less than kSlowThreshold, switches to position control for precise slow speed movement.
    public static double kSlowThreshold = NTDouble.create(2.5, "MotorDefault/kSlowThreshold", (val)->kSlowThreshold = val);
    public static double kSlowHysteresis = NTDouble.create(.2, "MotorDefault/kSlowHysteresis", (val)->kSlowHysteresis = val);
    public static double kSlowTransitionExtraSpin = NTDouble.create(.7,"MotorDfault/kSlowTransExtraspin",(val)->kSlowTransitionExtraSpin=val);

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

  public static class FunnelConstants {

    public static MyMotorType kMotorType = MyMotorType.NEO;
    public static int kFunnelCANID = 8;

    // Position control gains
    public static double kFunnelPosKp = (NTDouble.create(0.3, "Funnel/Position/kP",(val)->Funnel.getInstance().m_motorFunnel.withKP(val, Motor.GainSlot.POSITION)));
    public static double kFunnelPosKi = (NTDouble.create(0.002, "Funnel/Position/kI",(val)->Funnel.getInstance().m_motorFunnel.withKI(val, Motor.GainSlot.POSITION)));
    public static double kFunnelPosKd = (NTDouble.create(0, "Funnel/Position/kD",(val)->Funnel.getInstance().m_motorFunnel.withKD(val, Motor.GainSlot.POSITION)));
    public static double kFunnelPosKff = (NTDouble.create(0, "Funnel/Position/kFF",(val)->Funnel.getInstance().m_motorFunnel.withKFF(val, Motor.GainSlot.POSITION)));

    public static double kFunnelUp = 0.925; // Change once we can test
    public static double kFunnelDown = 0; // Change once we can test

    public static int kFunnelMotorCurrentLimit = (int)NTDouble.create(5,"Funnel/kCurrentLimit",(val) ->Funnel.getInstance().m_motorFunnel.withStatorLimit(val));
    public static double kFunnelAngleTolerance = Units.degreesToRadians(NTDouble.create(0.3, "Funnel/kFunnelAngleTolerance", val -> kFunnelAngleTolerance = Units.degreesToRadians(val)));

    public static double kFunnelGearRatio = 1;
    public static double kMaxSpeed = 1;
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
    public static boolean kEnabled = true;
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

  public static class ClimberConstants {
    public static MyMotorType kMotorType = MyMotorType.NEO;
    // TODO: Change later
    public static int kDigitalInputID = 2;
    public static int kCANID = 3;

    //public static int kCurrentInit = (int)NTDouble.create(3,"Climber/kCurrentInit",val->kCurrentInit=(int)val);
    public static int kMotorCurrentLimit = (int)NTDouble.create(50,"Climber/kCurrentLimit",(val) ->Climber.getInstance().m_motor.withStatorLimit(val));
    public static double kAngleTolerance = Units.degreesToRadians(NTDouble.create(3, "Climber/kClimberAngleTolerance", val -> kAngleTolerance = Units.degreesToRadians(val)));

    public static double kAngleIn = 0; // Change once we can test
    public static double kAngleOut = 0; // Change once we can test

    // TODO: Change values of Calibration Maps.
    public static double [] kCalibrationX = new double[]{0.507191, 0.581880, 0.829728, 0.989784}; //analog values 
    public static double [] kCalibrationY = new double[]{25, 0, -90, -148}; // degrees 
    
    public static double kGearRatio = 125;

    public static double kMaxSpeed = 1;
  }
  
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

      public static MyMotorType kDriveType = MyMotorType.KRAKEN;
      public static MyMotorType kSteerType = MyMotorType.NEO;
      
      public static double kCalibrationFrontLeft = (kDriveType == MyMotorType.KRAKEN ? 2487.0 : 1502.0);
      public static double kCalibrationFrontRight = (kDriveType == MyMotorType.KRAKEN ? 1104.0 : 2455.0);
      public static double kCalibrationRearLeft = (kDriveType == MyMotorType.KRAKEN ? 823.0 : 3634.0);
      public static double kCalibrationRearRight = (kDriveType == MyMotorType.KRAKEN ? 3949.0 : 2011.0);

      // Maximum Current Limits
      public static double kDrivingMotorCurrentLimit = NTDouble.create(40,"SwerveModule/kDrivingMotorCurrentLimit",val->DriveTrain.forEachSwerveModule((m)->{m.m_motorDrive.withStatorLimit((int)val);}));
      public static double kSteeringMotorCurrentLimit = NTDouble.create(5,"SwerveModule/kSteeringMotorCurrentLimit",val->DriveTrain.forEachSwerveModule((m)->{m.m_motorSteer.withStatorLimit((int)val);}));

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
      public static double kDriveKp  = NTDouble.create(switch(kDriveType){case KRAKEN->1.1; case NEO->0.064; default-> 0;},"SwerveModule/kDriveKp",val->DriveTrain.forEachSwerveModule((m)->m.m_motorDrive.withKP(val,Motor.GainSlot.SPEED)));
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
