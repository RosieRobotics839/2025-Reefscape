// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SystemLog extends SubsystemBase {

  private static SystemLog instance = new SystemLog();
  public static SystemLog getInstance(){
    return instance;
  }

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/System");
  final BooleanPublisher nt_DSonnected = table.getBooleanTopic("DSonnected").publish();
  final BooleanPublisher nt_isAutonomous = table.getBooleanTopic("isAutonomous").publish();
  final BooleanPublisher nt_isAutonomousEnabled = table.getBooleanTopic("isAutonomousEnabled").publish();
  final BooleanPublisher nt_isEnabled = table.getBooleanTopic("isEnabled").publish();
  final BooleanPublisher nt_isEStopped = table.getBooleanTopic("isEStopped").publish();
  final BooleanPublisher nt_isFMSAttached = table.getBooleanTopic("isFMSAttached").publish();
  final BooleanPublisher nt_isTeleop = table.getBooleanTopic("isTeleop").publish();
  final BooleanPublisher nt_isTeleopEnabled = table.getBooleanTopic("isTeleopEnabled").publish();
  final BooleanPublisher nt_isJoystickConnected0 = table.getBooleanTopic("isJoystickConnected0").publish();
  final BooleanPublisher nt_isJoystickConnected1 = table.getBooleanTopic("isJoystickConnected1").publish();
  final BooleanPublisher nt_getUserButton = table.getBooleanTopic("getUserButton").publish();

  /** Creates a new SystemLog. */
  public SystemLog() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    nt_DSonnected.set(DriverStation.isDSAttached());
    nt_isAutonomous.set(DriverStation.isAutonomous());
    nt_isAutonomousEnabled.set(DriverStation.isAutonomousEnabled());
    nt_isEnabled.set(DriverStation.isEnabled());
    nt_isEStopped.set(DriverStation.isEStopped());
    nt_isFMSAttached.set(DriverStation.isFMSAttached());
    nt_isTeleop.set(DriverStation.isTeleop());
    nt_isTeleopEnabled.set(DriverStation.isTeleopEnabled());
    nt_isJoystickConnected0.set(DriverStation.isJoystickConnected(0));
    nt_isJoystickConnected1.set(DriverStation.isJoystickConnected(1));
    nt_getUserButton.set(RobotController.getUserButton());

  }
}
