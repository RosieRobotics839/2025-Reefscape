// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EffectorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoreConstants;
import frc.robot.Constants.ScoreConstants.GamePieceSelected;
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
  public static boolean m_scoreLeft;

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
    }

  }

  static NTDouble nt_rStickXAxis = new NTDouble(0,table,"Accessory/rStickXAxis",(val)->{});
  static NTDouble nt_yStickXAxis = new NTDouble(0,table,"Accessory/yStickXAxis",(val)->{});
  static NTDouble nt_rStickYAxis = new NTDouble(0,table,"Accessory/rStickYAxis",(val)->{});
  static NTDouble nt_yStickYAxis = new NTDouble(0,table,"Accessory/yStickYAxis",(val)->{});
  static NTBoolean nt_scoreLeft = new NTBoolean(true,table,"scoreleft",(val)->{});

  public static class AccessoryButtons {
    public JoystickButton Intake, Outtake, ClimberIn, ClimberOut, GMPCS, StageDial0, StageDial1, StageDial2, StageDial3, StageDial4, LeftScore, RightScore, RS, Home;

    public Command waitForTarget = Commands.sequence(
      Commands.waitUntil(() -> {return Arm.getInstance().isAtPosition() && Elevator.getInstance().isAtPosition();})
    );

    public Command disableDirectControl(){
      return new InstantCommand(()->{getAccessoryInstance().m_directElevator = false; getAccessoryInstance().m_directArm = false;});
    }

    ScoreConstants.ScoreLevel m_level;
    public Command m_expel = EndEffector.getInstance().ExpelCommand(()->(m_level == ScoreLevel.TROUGH ? EffectorConstants.kTroughOuttakeSpeed : EffectorConstants.kOuttakeSpeed), ()->m_level==ScoreLevel.TROUGH);
      
    ScoreConstants.GamePieceSelected m_pieceSelected;

    // boolean that decides which game piece we are handling
    boolean isAlgaeSelected = false;  // Initially set to false (CORAL)

    NTBoolean nt_algaeSelected = new NTBoolean(false,table,"algaeSelected",(val)->{});


    public boolean toggleAlgae(){
      isAlgaeSelected = !isAlgaeSelected;
      nt_algaeSelected.set(isAlgaeSelected);
      return true;
    }

    AccessoryButtons(Controller controller){

      StageDial0 = new JoystickButton(controller, 1);  // Stage Dial Scoring Level 0 (Default/Human Player Intake)
      StageDial1 = new JoystickButton(controller, 2);  // Stage Dial Scoring Level 1 (Trough)
      StageDial2 = new JoystickButton(controller, 3);  // Stage Dial Scoring Level 2
      StageDial3 = new JoystickButton(controller, 4);  // Stage Dial Scoring Level 3
      StageDial4 = new JoystickButton(controller, 5);  // Stage Dial Scoring Level 4
      RightScore = new JoystickButton(controller, 6); // Scoring Right of Reef Face (Switch)
      LeftScore  = new JoystickButton(controller, 7); // Scoring Left of Reef Face (Switch)
      Intake     = new JoystickButton(controller, 8);  // Intake Button
      Outtake    = new JoystickButton(controller, 9);  // Expel Button (Outtake for those who don't know)
      ClimberOut = new JoystickButton(controller, 10);  // Brings Climber Out & Funnel Down
      ClimberIn  = new JoystickButton(controller, 11);  // Brings Climber In & Funnel Up
      GMPCS      = new JoystickButton(controller, 12);  // Game Piece Selector Button (Algae or Coral)

      //RS       = new JoystickButton(controller, 12); // Right Stick Click
      //Home     = new JoystickButton(controller, 13); // Home Button
      //DPadUp    = new POVButton(controller, 0);
      //DPadRight    = new POVButton(controller, 90);
      //DPadDown    = new POVButton(controller, 180);
      //DPadLeft    = new POVButton(controller, 270);

      // Initializing the selected game piece to be the default; coral.
      m_pieceSelected = ScoreConstants.GamePieceSelected.CORAL;

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
        Commands.sequence(
          waitForTarget.onlyIf(() -> m_pieceSelected == GamePieceSelected.CORAL), // Waiting for arm and elevator to reach target, will only run if coral is selected
          m_expel // Expelling either way, no matter algae or coral
        )
      );
      
      /* Setting Stage Dial Values */

    StageDial0.whileTrue(
      Commands.sequence(
        disableDirectControl(),
        new InstantCommand(()->m_level = ScoreLevel.FUNNEL),
        Elevator.getInstance().moveToLevelCommand(()->m_level),
        Arm.getInstance().moveToLevelCommand(()->m_level)
      )
    );
      
    StageDial1.debounce(0.2,DebounceType.kRising).whileTrue(
      Commands.sequence(
        disableDirectControl(),
        new InstantCommand(()->m_level = ScoreConstants.ScoreLevel.TROUGH),
        Elevator.getInstance().moveToLevelCommand(()->m_level),
        Arm.getInstance().moveToLevelCommand(()->m_level)
    ));

    StageDial2.debounce(0.2,DebounceType.kRising).whileTrue(
      Commands.sequence(
        disableDirectControl(),
        new InstantCommand(()->m_level = ScoreConstants.ScoreLevel.LEVEL2),
        Elevator.getInstance().moveToLevelCommand(()->m_level),
        Arm.getInstance().moveToLevelCommand(()->m_level)
    ));

    StageDial3.debounce(0.2,DebounceType.kRising).whileTrue(
      Commands.sequence(
        disableDirectControl(),
        new InstantCommand(()->m_level = ScoreConstants.ScoreLevel.LEVEL3),
        Elevator.getInstance().moveToLevelCommand(()->m_level),
        Arm.getInstance().moveToLevelCommand(()->m_level)
    ));

    StageDial4.whileTrue(
      Commands.sequence(
        disableDirectControl(),
        new InstantCommand(()->m_level = ScoreConstants.ScoreLevel.LEVEL4),
        Elevator.getInstance().moveToLevelCommand(()->m_level),
        Arm.getInstance().moveToLevelCommand(()->m_level)
    ));


      /* Side Positioning for Scoring */

      LeftScore.onTrue(
        Commands.sequence(
          new InstantCommand(()->m_scoreLeft = true)
        )
      );
      RightScore.onTrue(
        Commands.sequence(
          new InstantCommand(()->m_scoreLeft = false)
        )
      );

      /* Algae/Coral Selector */

      GMPCS.onTrue(
          Commands.sequence(
            Commands.waitUntil(() -> {return toggleAlgae();}),
            Commands.waitUntil(() -> {
              if (isAlgaeSelected) {
                  m_pieceSelected = ScoreConstants.GamePieceSelected.ALGAE;
              } else {
                  m_pieceSelected = ScoreConstants.GamePieceSelected.CORAL;
              }
              return true;
            })
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

    Translation2d Rstick = new Translation2d(this.getRightX(),this.getRightY());
    Rstick = VectorUtils.deadband(Rstick,0.1,1);
    Ry = Rstick.getY();
    Rx = Rstick.getX();
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
    nt_scoreLeft.set(m_scoreLeft);
  }
}