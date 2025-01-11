// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.utils.VectorUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.kDriveTrain.DriveConstants;
public class Controller extends XboxController {

  public static double forward;
  public static double left;
  public static double extend;
  public static double rotate;
  public static double grab;
  public static Integer speedSelect = DriveConstants.kMaxSpeedDefault;

  public static Controller driveController = new Controller(0);  
  public static Controller accessoryController = new Controller(1);
  
  public static AccessoryButtons accessoryButtons = new AccessoryButtons(accessoryController);

  public static Controller getDriveInstance(){
    return driveController;
  }
  public static Controller getAccessoryInstance(){
    return accessoryController;
  }

  public class ControllerButtons {
    public JoystickButton X, O, Triangle, Square, LB, RB, LT, RT, Select, Start, LS, RS, Home;
    ControllerButtons(Controller controller){
      X        = new JoystickButton(controller, 1);  // Orange Square
      O        = new JoystickButton(controller, 2);  // Blue X
      Triangle = new JoystickButton(controller, 3);  // Red Circle
      Square   = new JoystickButton(controller, 4);  // Green Triangle
      LB       = new JoystickButton(controller, 5);  // Left Shoulder Button
      RB       = new JoystickButton(controller, 6);  // Right Shoulder Button
      LT       = new JoystickButton(controller, 7);  // Left Trigger
      RT       = new JoystickButton(controller, 8);  // Right Trigger
      Select   = new JoystickButton(controller, 9);  // Select
      Start    = new JoystickButton(controller, 10); // Start
      LS       = new JoystickButton(controller, 11); // Left Stick Click
      RS       = new JoystickButton(controller, 12); // Right Stick Click
      Home     = new JoystickButton(controller, 13); // Home Button
    }
  }
  
  public static Integer rangeLimit(Integer index, Integer min, Integer max){
    return Math.max(min,Math.min(max,index));
  }

  public static class DriveButtons {

    Integer m_speedSelector = DriveConstants.kMaxSpeedDefault;
    DriveTrain driveTrain = DriveTrain.getInstance(); 
    Autonomous m_autonomous = Autonomous.getInstance();
    static ControllerButtons buttons; 

    DriveButtons(){
      buttons.X.onTrue(new InstantCommand(() -> {
        m_speedSelector = rangeLimit(++m_speedSelector, 0, DriveConstants.kMaxSpeedMetersPerSecond.length-1);
        driveTrain.setMaxSpeed(DriveConstants.kMaxSpeedMetersPerSecond[m_speedSelector]);
      }));
      buttons.Square.onTrue(new InstantCommand(() -> {
        m_speedSelector = rangeLimit(--m_speedSelector, 0, DriveConstants.kMaxSpeedMetersPerSecond.length-1);
        driveTrain.setMaxSpeed(DriveConstants.kMaxSpeedMetersPerSecond[m_speedSelector]);
      }));
      buttons.Triangle.onTrue(new InstantCommand(()->{
        m_autonomous.aimAtPoint(new Pose2d(Units.feetToMeters(3),Units.feetToMeters(1),new Rotation2d(0)));
      }));
      buttons.Triangle.onFalse(new InstantCommand(()->{
        m_autonomous.stopAiming();
      }));
      //buttons.RS.onTrue(new InstantCommand(() -> {driveTrain.m_fieldCentricDriving = !driveTrain.m_fieldCentricDriving;}));
    }

  }

  public static class AccessoryButtons {
    public JoystickButton Orange, Blue, Red, Green, LB, RB, LT, RT, Select, Start, LS, RS, Home;
    public POVButton DPadUp, DPadRight, DPadDown, DPadLeft;
    AccessoryButtons(Controller controller){
      Orange = new JoystickButton(controller, 1);  // Orange Square
      Blue   = new JoystickButton(controller, 2);  // Blue X
      Red    = new JoystickButton(controller, 3);  // Red Circle
      Green  = new JoystickButton(controller, 4);  // Green Triangle
      LB     = new JoystickButton(controller, 5);  // Left Shoulder Button
      RB     = new JoystickButton(controller, 6);  // Right Shoulder Button
      LT     = new JoystickButton(controller, 7);  // Left Trigger
      RT     = new JoystickButton(controller, 8);  // Right Trigger
      Select = new JoystickButton(controller, 9);  // Select
      Start  = new JoystickButton(controller, 10); // Start
      LS     = new JoystickButton(controller, 11); // Left Stick Click
      RS     = new JoystickButton(controller, 12); // Right Stick Click
      Home   = new JoystickButton(controller, 13); // Home Button
      DPadUp    = new POVButton(controller, 0);
      DPadRight    = new POVButton(controller, 90);
      DPadDown    = new POVButton(controller, 180);
      DPadLeft    = new POVButton(controller, 270);

    // DriveTrain driveTrain = DriveTrain.getInstance(); 
    // Autonomous m_autonomous = Autonomous.getInstance();
    // IntakeShooter intakeShooter = IntakeShooter.getInstance();
    // Climber climberL = Climber.right();
    // Climber climberR = Climber.left();
    // static AccessoryButtons buttons; 
    // AccessoryButton(Controller controller) {

      /* Set climber values(?) */

      LB.whileTrue(new RepeatCommand(new InstantCommand(() -> {
        Climber.setAverageHeight(Climber.getAverageTargetHeight() - ClimberConstants.kMaxSpeed * 0.02);
      })));
      RB.whileTrue(new RepeatCommand(new InstantCommand(() -> {
        Climber.setAverageHeight(Climber.getAverageTargetHeight() + ClimberConstants.kMaxSpeed * 0.02);
      })));
      LT.whileTrue(new RepeatCommand(new InstantCommand(() -> {
        Climber.setOffset(Climber.getTargetOffset() - ClimberConstants.kMaxSpeed * 0.02);
      })));
      RT.whileTrue(new RepeatCommand(new InstantCommand(() -> {
        Climber.setOffset(Climber.getTargetOffset() + ClimberConstants.kMaxSpeed * 0.02);
      })));

      LS.and(RS).onTrue(new InstantCommand(() -> IntakeShooter.getInstance().setAngleInverted()));
      /* Set shooter amp / speaker */

      Blue.onTrue(new InstantCommand(() -> {
        // TODO: Set amp shoot & change to that angle
        IntakeShooter.getInstance().setShooterSpeedRatio(1);
      }));
      Blue.onFalse(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterSpeedRatio(0);
      }));
      Green.onTrue(new InstantCommand(() -> {
        // TODO: Set speaker shoot & change to that angle
        IntakeShooter.getInstance().setShooterSpeedRatio(1);
      }));
      Green.onFalse(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterSpeedRatio(0);
      }));
      
      
      /* Set intake in / out */
      
      Orange.onTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().intakeSequence = false;
        IntakeShooter.getInstance().intakeNote.cancel();
        IntakeShooter.getInstance().setIntakeSpeedRatio(0.75);
      }));
      Orange.onFalse(new InstantCommand(() -> {
        IntakeShooter.getInstance().setIntakeSpeedRatio(0);
      }));
      Red.onTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().intakeSequence = false;
        IntakeShooter.getInstance().intakeNote.cancel();
        IntakeShooter.getInstance().setIntakeSpeedRatio(-1);
      }));
      Red.onFalse(new InstantCommand(() -> {
        IntakeShooter.getInstance().setIntakeSpeedRatio(0);
      }));


      /* Set arm positions. */

      DPadDown.onTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Ground);
      }));
      DPadLeft.onTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Amp);
      }));
      DPadUp.onTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up);
      }));
      DPadRight.onTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Speaker);
      }));
      DPadRight.whileTrue(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterAngle(IntakeShooter.getInstance().m_speakerAngle);
      }));
      DPadRight.onFalse(new InstantCommand(() -> {
        IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Speaker);
      }));
    }
  }

  public Controller(int port) {
    super(port);
  }

  public void Translate() {
    // TODO: Move shooter angle on the left joystick.
    Translation2d Lstick = new Translation2d(this.getLeftX(),-this.getLeftY());
    Lstick = VectorUtils.deadband(Lstick,0.1,1);
    forward = Lstick.getY(); // Forward is Positive consistent the FRC field coordinate system
    left = -Lstick.getX();   // Left is Positive consistent the FRC field coordinate system
    
    Translation2d Rstick = new Translation2d(this.getRightX(),this.getRightY());
    Rstick = VectorUtils.deadband(Rstick,0.1,1);
    rotate = -Rstick.getX(); // Counter Clockwise is Positive consistent the FRC field coordinate system
    

    if (DriverStation.isTeleopEnabled()){
      IntakeShooter.getInstance().setShooterAngle(IntakeShooter.getInstance().getAngleTarget() + ShooterConstants.kManualAngleSpeed * 0.02 * forward);
    }
    // getLeftX()
    // getLeftY()
    // getRightX()
    // getRightY()
  }

  public void armThing() {
    if (this.getLeftY() < 0.01 && this.getLeftY() > -0.01) {
      rotate = 0;
    } else {
      rotate = -this.getLeftY();
    }
    
    if (this.getRightY() < 0.01 && this.getRightY() > -0.01) {
      extend = 0;
    } else {
      extend = -this.getRightY();
    }
    grab = -this.getLeftTriggerAxis() + this.getRightTriggerAxis();
  }
}