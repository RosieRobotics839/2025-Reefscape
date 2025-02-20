// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.utils.VectorUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    public JoystickButton Intake, Outtake, ClimberIn, ClimberOut, GMPCS, StageDial1, StageDial2, StageDial3, StageDial4, LeftScore, RightScore, RS, Home;
    public POVButton DPadUp, DPadRight, DPadDown, DPadLeft;
    AccessoryButtons(Controller controller){
      Intake     = new JoystickButton(controller, 1);  // Intake Button
      Outtake    = new JoystickButton(controller, 2);  // Expel Button (Outtake for those who don't know)
      ClimberIn  = new JoystickButton(controller, 3);  // Brings Climber In & Funnel Up
      ClimberOut = new JoystickButton(controller, 4);  // Brings Climber Out & Funnel Down
      GMPCS      = new JoystickButton(controller, 5);  // Game Piece Selector Button (Algae or Coral)
      StageDial1 = new JoystickButton(controller, 6);  // Stage Dial Scoring Level 1 (Trough)
      StageDial2 = new JoystickButton(controller, 7);  // Stage Dial Scoring Level 2
      StageDial3 = new JoystickButton(controller, 8);  // Stage Dial Scoring Level 3
      StageDial4 = new JoystickButton(controller, 9);  // Stage Dial Scoring Level 4
      LeftScore  = new JoystickButton(controller, 10); // Scoring Left of Reef Face (Switch)
      RightScore = new JoystickButton(controller, 11); // Scoring Right of Reef Face (Switch)
      //RS       = new JoystickButton(controller, 12); // Right Stick Click
      //Home     = new JoystickButton(controller, 13); // Home Button
      //DPadUp    = new POVButton(controller, 0);
      //DPadRight    = new POVButton(controller, 90);
      //DPadDown    = new POVButton(controller, 180);
      //DPadLeft    = new POVButton(controller, 270);

      /* Run Climber Command Sequences */

      ClimberIn.onTrue(
        Commands.sequence(
          Funnel.getInstance().FunnelUpCommand,
          Climber.getInstance().ClimberInCommand
        )
      );

      ClimberOut.onTrue(
        Commands.sequence(
          Funnel.getInstance().FunnelDownCommand,
          Climber.getInstance().ClimberOutCommand
        )
      );

      /* Intake and Outtake Command Sequences */

      Intake.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );
      Outtake.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );
      
      /* Stage Dial Command Sequences */
      
      StageDial1.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );
      StageDial2.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );
      StageDial3.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );
      StageDial4.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );


      /* Side Positioning for Scoring */

      LeftScore.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );
      RightScore.onTrue(
        Commands.sequence(
          // TODO: Sequence Commands once branches tested and merged
        )
      );

      /* Algae/Coral Selector */

      GMPCS.onTrue(
        Commands.sequence(
        // TODO: Add code to select Algae
        )
      );
      GMPCS.onFalse(
        Commands.sequence(
        // TODO: Add code to select Coral
        )
      );
    } 
  }

  public Controller(int port) {
    super(port);
  }

  public void Translate() {
    Translation2d Lstick = new Translation2d(this.getLeftX(),-this.getLeftY());
    Lstick = VectorUtils.deadband(Lstick,0.1,1);
    forward = Lstick.getY(); // Forward is Positive consistent the FRC field coordinate system
    left = -Lstick.getX();   // Left is Positive consistent the FRC field coordinate system
    
    Translation2d Rstick = new Translation2d(this.getRightX(),this.getRightY());
    Rstick = VectorUtils.deadband(Rstick,0.1,1);
    rotate = -Rstick.getX(); // Counter Clockwise is Positive consistent the FRC field coordinate system
  }

  /* if (DriverStation.isTeleopEnabled()){
      IntakeShooter.getInstance().setShooterAngle(IntakeShooter.getInstance().getAngleTarget() + ShooterConstants.kManualAngleSpeed * 0.02 * forward);
    }
    // getLeftX()
    // getLeftY()
    // getRightX()
    // getRightY()
  } */

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