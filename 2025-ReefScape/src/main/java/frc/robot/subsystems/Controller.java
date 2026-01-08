// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTDouble;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScoreConstants;
import frc.robot.Constants.ScoreConstants.ReefAlignment;
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
      storeLast();
    }

    Translate();

    nt_yStickXAxis.set(Lx);
    nt_yStickYAxis.set(Ly);
  }
}