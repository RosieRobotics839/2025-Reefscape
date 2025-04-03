// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.KrakenOrchestra;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTDouble;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EffectorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScoreConstants;
import frc.robot.Constants.ScoreConstants.ReefAlignment;
import frc.robot.Constants.ScoreConstants.ScoreLevel;
import frc.robot.Constants.kDriveTrain.DriveConstants;
public class Controller extends XboxController {
  
  public Controller(int port) {
    super(port);
  }

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Controller");

  public double Ly, Lx, Ly_pre, Lx_pre;
  public double Ry, Rx, Ry_pre, Rx_pre;
  public boolean m_directElevator = false;
  public boolean m_directArm = false;
  public static ScoreConstants.ReefAlignment m_reefAlign = ReefAlignment.LEFT;

  public static Integer speedSelect = DriveConstants.kMaxSpeedDefault;

  public static Controller driveController = new Controller(0);
  public static Controller accessoryController = new Controller(1);
  
  public static AccessoryButtons accessoryButtons = new AccessoryButtons(accessoryController);
  public static KrakenOrchestra m_orchestra = KrakenOrchestra.getInstance();

  public static Controller getDriveInstance(){
    return driveController;
  }
  public static Controller getAccessoryInstance(){
    return accessoryController;
  }
  public static AccessoryButtons getAccessoryButtonsInstance(){
    return accessoryButtons;
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

    public DriveButtons(CommandXboxController controller) {
      controller.back().onTrue(new InstantCommand(() -> {
        m_speedSelector = rangeLimit(++m_speedSelector, 0, DriveConstants.kMaxSpeedMetersPerSecond.length-1);
        DriveTrain.getInstance().setMaxSpeed(DriveConstants.kMaxSpeedMetersPerSecond[m_speedSelector]);
      }));
      controller.start().onTrue(new InstantCommand(() -> {
        m_speedSelector = rangeLimit(--m_speedSelector, 0, DriveConstants.kMaxSpeedMetersPerSecond.length-1);
        DriveTrain.getInstance().setMaxSpeed(DriveConstants.kMaxSpeedMetersPerSecond[m_speedSelector]);
      }));
      controller.rightBumper().onTrue(new InstantCommand(() -> {
        DriveTrain.getInstance().setTargetHeading(DriveTrain.getInstance().getTargetHeading()+Units.degreesToRadians(90)); // CCW 90 Degrees
      }));
      controller.leftBumper().onTrue(new InstantCommand(() -> {
        DriveTrain.getInstance().setTargetHeading(DriveTrain.getInstance().getTargetHeading()-Units.degreesToRadians(90)); // CW 90 Degrees
      }));
      controller.leftTrigger(0.2).onTrue(new InstantCommand(()->
      AutoCommands.DriveReefOffset()
      ));
      controller.rightStick().onTrue(new InstantCommand(() -> {
        OperatorConstants.kFieldCentricDriving = !OperatorConstants.kFieldCentricDriving;
      }));
      controller.leftStick().onTrue(new InstantCommand(() -> {
        DriveTrain.getInstance().setMaxRotate(DriveConstants.kGetItOffMeRotationSpeed);
      }));
      controller.leftStick().onFalse(new InstantCommand(() -> {
        DriveTrain.getInstance().setMaxRotate(Units.degreesToRadians(75));
      }));
    }

    Integer m_speedSelector = DriveConstants.kMaxSpeedDefault;
    DriveButtons(){
      
    }

  }

  static NTDouble nt_rStickXAxis = new NTDouble(0,table,"Accessory/rStickXAxis",(val)->{});
  static NTDouble nt_yStickXAxis = new NTDouble(0,table,"Accessory/yStickXAxis",(val)->{});
  static NTDouble nt_rStickYAxis = new NTDouble(0,table,"Accessory/rStickYAxis",(val)->{});
  static NTDouble nt_yStickYAxis = new NTDouble(0,table,"Accessory/yStickYAxis",(val)->{});
  static StringPublisher nt_scoreAlignment = table.getStringTopic("scoreAlignment").publish();

  public static class AccessoryButtons {
    public JoystickButton Intake, Outtake, ClimberIn, ClimberOut, ByeAlgae, StageDial0, StageDial1, StageDial2, StageDial3, StageDial4, LeftScore, RightScore, CenterAlign, Home;
    //public POVButton DPadUp, DPadRight, DPadDown, DPadLeft;

    public Command disableDirectControl(){
      return new InstantCommand(()->{getAccessoryInstance().m_directElevator = false; getAccessoryInstance().m_directArm = false;});
    }

    ScoreConstants.ScoreLevel m_level;
    private double expelSpeed(){
      switch (m_level) {
        case TROUGH:
          return EffectorConstants.kTroughOuttakeSpeed;
        case CORAL2:
        case CORAL3:
          return EffectorConstants.kOuttakeFastSpeed;
        default:
          return EffectorConstants.kOuttakeSpeed;
      }
    }
    public Command m_expel = EndEffector.getInstance().ExpelCommand(()->expelSpeed(), ()->m_level==ScoreLevel.TROUGH);

    AccessoryButtons(Controller controller){

      StageDial0  = new JoystickButton(controller, 1); // Stage Dial Scoring Level 0 (Default/Human Player Intake)
      StageDial1  = new JoystickButton(controller, 2); // Stage Dial Scoring Level 1 (Trough)
      StageDial2  = new JoystickButton(controller, 3); // Stage Dial Scoring Level 2
      StageDial3  = new JoystickButton(controller, 4); // Stage Dial Scoring Level 3
      StageDial4  = new JoystickButton(controller, 5); // Stage Dial Scoring Level 4
      RightScore  = new JoystickButton(controller, 6); // Scoring Right of Reef Face (Switch)
      LeftScore   = new JoystickButton(controller, 7); // Scoring Left of Reef Face (Switch)
      Intake      = new JoystickButton(controller, 8); // Intake Button
      Outtake     = new JoystickButton(controller, 9); // Expel Button (Outtake for those who don't know)
      ClimberOut  = new JoystickButton(controller, 10); // Brings Climber Out & Funnel Down
      ClimberIn   = new JoystickButton(controller, 11); // Brings Climber In & Funnel Up
      ByeAlgae    = new JoystickButton(controller, 12); // Game Piece Selector Button (Algae or Coral)
      CenterAlign = new JoystickButton(controller, 13); // Center Alignment

      //Home     = new JoystickButton(controller, 13); // Home Button
      //DPadUp     = new POVButton(controller, 0);
      //DPadRight    = new POVButton(controller, 90);
      //DPadDown    = new POVButton(controller, 180);
      //DPadLeft    = new POVButton(controller, 270);

      /* Run Climber Command Sequences */
      ClimberIn.and(()->!ClimberOut.getAsBoolean()).debounce(0.06,DebounceType.kRising).whileTrue(
          // Funnel retracts automatically when climber comes in
          Climber.getInstance().ClimberInCommand
      );

      ClimberOut.and(()->!ClimberIn.getAsBoolean()).debounce(0.06,DebounceType.kRising).whileTrue(
          Climber.getInstance().ClimberOutCommand
      );

      // Put funnel back down if both buttons are pressed.
      ClimberIn.and(()->ClimberOut.getAsBoolean()).whileTrue(Funnel.getInstance().FunnelDownCommand);

      /* Intake and Outtake Command Sequences */
      Intake.toggleOnTrue(
        EndEffector.getInstance().IntakeCommand()
      );

      Outtake.onTrue(
        m_expel // Expelling either way, no matter algae or coral
      );
      
      /* Setting Stage Dial Values */

    StageDial0.whileTrue(
      Commands.either(
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.ALGAE0),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        ),
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.FUNNEL),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
      ,()->m_reefAlign == ReefAlignment.CENTER)
    );
      
    StageDial1.debounce(0.2,DebounceType.kRising).whileTrue(
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.TROUGH),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
    );

    StageDial2.debounce(0.2,DebounceType.kRising).whileTrue(
      Commands.either(
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.ALGAE2),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        ),
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.CORAL2),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
      ,()->m_reefAlign == ReefAlignment.CENTER)
    );

    StageDial3.debounce(0.2,DebounceType.kRising).whileTrue(
      Commands.either(
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.ALGAE3),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        ),
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.CORAL3),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
      ,()->m_reefAlign == ReefAlignment.CENTER)
    );

    StageDial4.whileTrue(
      Commands.either(
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.ALGAE4),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        ),
        Commands.sequence(
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreLevel.CORAL4),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
      ,()->m_reefAlign == ReefAlignment.CENTER)
    );

      /* Side Positioning for Scoring */

      LeftScore.onTrue(
        Commands.sequence(
          new InstantCommand(()->m_reefAlign = ReefAlignment.LEFT),
          new InstantCommand(()->nt_scoreAlignment.set("LEFT")),
          disableDirectControl(),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
      );
      RightScore.onTrue(
        Commands.sequence(
          new InstantCommand(()->m_reefAlign = ReefAlignment.RIGHT),
          new InstantCommand(()->nt_scoreAlignment.set("RIGHT")),
          disableDirectControl(),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
      );

      CenterAlign.debounce(0.3).onTrue(
        Commands.sequence(
          new InstantCommand(()->m_reefAlign = ReefAlignment.CENTER),
          new InstantCommand(()->nt_scoreAlignment.set("CENTER")),
          disableDirectControl(),
          Elevator.getInstance().moveToLevelCommand(()->m_level),
          Arm.getInstance().moveToLevelCommand(()->m_level)
        )
      );

      /* Algae Button */

      ByeAlgae.onTrue(
        Commands.sequence(
          disableDirectControl(),
          AutoCommands.BargeFling()
        
        /*
        Commands.sequence(
        
          disableDirectControl(),
          new InstantCommand(()->m_level = ScoreConstants.ScoreLevel.ALGAE),
          Arm.getInstance().moveToLevelCommand(()->m_level)*/
        )  
      );
    } 
  }

  private void storeLast(){
    Ly_pre = Ly;
    Lx_pre = Lx;
  }

  public void Translate() {
    Translation2d Lstick = new Translation2d(this.getLeftX(),-this.getLeftY());
    Lstick = VectorUtils.deadband(Lstick,0.1,1);
    Ly = Lstick.getY();
    Lx = Lstick.getX();
    if (getAxisCount() > 2){
      Translation2d Rstick = new Translation2d(this.getRightX(),this.getRightY());
      Rstick = VectorUtils.deadband(Rstick,0.1,1);
      Ry = Rstick.getY();
      Rx = -Rstick.getX();
    }
  }
  
  public void accessoryPeriodic(){
    
    if (DriverStation.isDisabled()){
      m_directArm = false;
      m_directElevator = false;
      storeLast();
    }

    Translate();
    if (Math.abs(Lx-Lx_pre) > 0.05){
      m_directElevator = true;
    }

    if (Math.abs(Ly-Ly_pre) > 0.05){
      m_directArm = true;
    }

    // store manual control positions, to check if they have moved after automated control.
    if (m_directArm){
      Ly_pre = Ly;
    }
    if (m_directElevator){
      Lx_pre = Lx;
    }

    if (m_directElevator){
      Elevator.getInstance().setPosition((ElevatorConstants.kMaxHeight - ElevatorConstants.kMinHeight)*(Lx+1.0)/2.0 + ElevatorConstants.kMinHeight);
    }

    if (m_directArm){
      Arm.getInstance().setPosition((ArmConstants.kAngleMax - ArmConstants.kAngleMin)*(-Ly+1.0)/2.0 + ArmConstants.kAngleMin);
    }

    nt_yStickXAxis.set(Lx);
    nt_yStickYAxis.set(Ly);
  }
}