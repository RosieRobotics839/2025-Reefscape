// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.utils.VectorUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class FlightStick extends Joystick {

  public static double forward;
  public static double left;
  public static double rotate;  
  public static double turboscale = 1.0;
  public double slider = 0;

  public static boolean m_blueAlly = true;

  public static FlightStick driveController = new FlightStick(0);
  
  public static DriveButtons driveButtons = new DriveButtons(driveController);

  public static boolean m_preventDriverRotation = false;

  public static FlightStick getDriveInstance(){
    return driveController;
  }

  public static Integer rangeLimit(Integer index, Integer min, Integer max){
    return Math.max(min,Math.min(max,index));
  }
  public static class DriveButtons {

    private Integer m_speedSelector = DriveConstants.kMaxSpeedDefault;

    public JoystickButton Trigger,Top2Btn,Top3Btn,Top4Btn,Top5Btn,Top6Btn,Btm7Btn,Btm8Btn,Btm9Btn,Btm10Btn,Btm11Btn,Btm12Btn;
    DriveButtons(FlightStick controller){
      Trigger   = new JoystickButton(controller, 1);  // Trigger on Joystick
      Top2Btn   = new JoystickButton(controller, 2);  // Button on side of main stick
      Top3Btn   = new JoystickButton(controller, 3);  // On top part of stick
      Top4Btn   = new JoystickButton(controller, 4);  //  |
      Top5Btn    = new JoystickButton(controller, 5);  // |
      Top6Btn    = new JoystickButton(controller, 6);  // ↓
      Btm7Btn  = new JoystickButton(controller, 7);  // On bottom part of stick
      Btm8Btn  = new JoystickButton(controller, 8);  //  |
      Btm9Btn  = new JoystickButton(controller, 9);  //  |
      Btm10Btn  = new JoystickButton(controller, 10); // |
      Btm11Btn  = new JoystickButton(controller, 11); // |
      Btm12Btn  = new JoystickButton(controller, 12); // ↓

      //Trigger.onTrue(new InstantCommand(() -> {turboscale=1.5;}));
      //Trigger.onFalse(new InstantCommand(() -> {turboscale=1.0;}));
  
      /* Change speed modes */

      Trigger.onTrue(new InstantCommand(() -> {
        m_speedSelector = rangeLimit(++m_speedSelector, 0, DriveConstants.kMaxSpeedMetersPerSecond.length-1);
        DriveTrain.getInstance().setMaxSpeed(DriveConstants.kMaxSpeedMetersPerSecond[m_speedSelector]);
        DriveTrain.getInstance().setMaxRotate(DriveConstants.kMaxRotationVelocity[m_speedSelector]);
      }));
      Top2Btn.onTrue(new InstantCommand(() -> {
        m_speedSelector = rangeLimit(--m_speedSelector, 0, DriveConstants.kMaxSpeedMetersPerSecond.length-1);
        DriveTrain.getInstance().setMaxSpeed(DriveConstants.kMaxSpeedMetersPerSecond[m_speedSelector]);
        DriveTrain.getInstance().setMaxRotate(DriveConstants.kMaxRotationVelocity[m_speedSelector]);
      }));


      /* Rotate 90 degrees */

      Top5Btn.onTrue(new InstantCommand(()->{
        DriveTrain.getInstance().setTargetHeading(DriveTrain.getInstance().getTargetHeading()+Units.degreesToRadians(90)); // CCW 90 Degrees
      }));
      Top6Btn.onTrue(new InstantCommand(()->{
        DriveTrain.getInstance().setTargetHeading(DriveTrain.getInstance().getTargetHeading()-Units.degreesToRadians(90)); // CW 90 Degrees
      }));

      // Btm4Btn.whileTrue(new InstantCommand(() -> {
      //   IntakeShooter.getInstance().setIntakeSpeed(1);
      // }));
      // Btm4Btn.onFalse(new InstantCommand(() -> {
      //   IntakeShooter.getInstance().setIntakeSpeed(0);
      // }));
      // Btm5Btn.onTrue(new InstantCommand(() -> {
      //   IntakeShooter.getInstance().setShooterSpeed(1);
      // }));
      // Btm5Btn.onFalse(new InstantCommand(() -> {
      //   IntakeShooter.getInstance().setShooterSpeed(0);
      // }));


        /* Aim at Speaker */

      Btm8Btn.onTrue(new InstantCommand(()->{
        Autonomous.getInstance().aimAtPoint(Vision.getInstance().aprilTagFieldLayout.getTagPose(AutonomousCommands.speakerTag()).get().toPose2d(),Units.degreesToRadians(180));
      }));
      Btm8Btn.onFalse(new InstantCommand(()->{
        Autonomous.getInstance().stopAiming();
      }));


      Btm7Btn.onTrue(new InstantCommand(()->{
        Autonomous.getInstance().aimAtPoint(Vision.getInstance().aprilTagFieldLayout.getTagPose(AutonomousCommands.ampTag()).get().toPose2d(),Units.degreesToRadians(180));
      }));
      Btm7Btn.onFalse(new InstantCommand(()->{
        Autonomous.getInstance().stopAiming();
      }));
      /* Go to HP / Amp */

      Top4Btn.onTrue(new InstantCommand(()->{
        PathPlanning.getInstance().navigateTo(new Pose2d(PathPlanning.AprilTagAtDistance(5,Units.feetToMeters(2)).getTranslation(),new Rotation2d(Units.degreesToRadians(-90))));
        IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Amp);
      }));
      Top3Btn.onTrue(new InstantCommand(()->{
        PathPlanning.getInstance().navigateTo(new Pose2d(PathPlanning.AprilTagAtDistance(4,Units.feetToMeters(6)).getTranslation(),new Rotation2d(Units.degreesToRadians(180))));
        IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Speaker);
      }));


      /* Swap between field centric and proportional */

      Btm9Btn.onTrue(new InstantCommand(() -> {
        OperatorConstants.kFieldCentricDriving = !OperatorConstants.kFieldCentricDriving;
      }));
      Btm11Btn.onTrue(new InstantCommand(() -> {
        DriveTrain.getInstance().setMaxRotate(DriveConstants.kGetItOffMeRotationSpeed);
      }));
      Btm11Btn.onFalse(new InstantCommand(() -> {
        DriveTrain.getInstance().setMaxRotate(Units.degreesToRadians(75));
      }));


      /* Intake in */
      Btm10Btn.onTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().setIntakeSpeedRatio(1);
      }));
      Btm10Btn.onFalse(new InstantCommand(() -> {
        IntakeShooter.getInstance().setIntakeSpeedRatio(0);
      }));
      Btm12Btn.whileTrue(new RepeatCommand(new InstantCommand(() -> {
        m_preventDriverRotation = true;
        Autonomous.getInstance().aimAtNote();
      })));
      Btm12Btn.onFalse(new InstantCommand(() -> {
        m_preventDriverRotation = false;
        Vision.getInstance().unlockTarget();
      }));
    }
  }

  public FlightStick(int port) {
    super(port);
  }

  public void Translate() {
   
    double flipField = 1.0;
    if (!m_blueAlly){
      flipField = -1.0;
    }
    
    slider = this.getThrottle();

    Translation2d Lstick = new Translation2d(-this.getX(),-this.getY());
    Lstick = VectorUtils.deadband(Lstick,0.1,1);
    forward = flipField * turboscale * Lstick.getY();  // Forward is Positive
    left = flipField * Lstick.getX();               // Left is Positive
    
    Translation2d Rstick = new Translation2d(this.getZ(),0);
    Rstick = VectorUtils.deadband(Rstick,0.35,1);
    rotate = -Rstick.getX();             // CCW is Positive

  }
}