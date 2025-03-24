// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.FlightStick;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PathPlanning;
import frc.robot.subsystems.Vision;
import frc.utils.Action;
import frc.utils.KrakenOrchestra;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SystemLog;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  public Command m_autonomousCommand;
  public SystemLog m_systemlog = SystemLog.getInstance();
  public DriveTrain m_drivetrain = DriveTrain.getInstance();
  public Gyro m_gyro = Gyro.getInstance();
  public LED m_led = LED.getInstance();
  public PoseEstimator m_poseestimator = PoseEstimator.getInstance();
  public PathPlanning m_pathplanning = PathPlanning.getInstance();
  public Vision m_vision = Vision.getInstance();
  public Autonomous m_autonomous = Autonomous.getInstance();
  public EndEffector m_endeffector = EndEffector.getInstance();
  public Arm m_arm = Arm.getInstance();
  public Climber m_climber = Climber.getInstance();
  public Funnel m_funnel = Funnel.getInstance();
  public KrakenOrchestra m_orchestra = KrakenOrchestra.getInstance();

  Alliance myAlliance = Alliance.Red;

  NetworkTable nt_smartdashboard = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  {
    if (Robot.isSimulation()){
      nt_smartdashboard.getStringTopic("Auto Selector").publish().set("Build Your Own A(uto)dventure");
      nt_smartdashboard.getStringTopic("DB/String 0").publish().set("Get.Left");
      nt_smartdashboard.getStringTopic("DB/String 1").publish().set("Score.SS.Left.4");
      nt_smartdashboard.getStringTopic("DB/String 2").publish().set("Get.Left");
      nt_smartdashboard.getStringTopic("DB/String 3").publish().set("Score.SS.Right.4");
      nt_smartdashboard.getStringTopic("DB/String 4").publish().set("");
      nt_smartdashboard.getStringTopic("DB/String 5").publish().set("");
      nt_smartdashboard.getStringTopic("DB/String 6").publish().set("");
      nt_smartdashboard.getStringTopic("DB/String 7").publish().set("");
      nt_smartdashboard.getStringTopic("DB/String 8").publish().set("");
      nt_smartdashboard.getStringTopic("DB/String 9").publish().set("");
    }
  }
  public boolean changedAlly = true;
  private boolean orchestraIsPlaying = false;
  public String selectedChoice;
  public String selectedSong;
  private static final String START_MUSIC = "playing";
  private static final String STOP_MUSIC = "notPlaying";

  private Debouncer m_recording = new Debouncer(10, Debouncer.DebounceType.kFalling);
  private Action m_recordTrigger = new Action(false).onTrue(()->DataLogManager.start()).onFalse(()->DataLogManager.stop());
  private SendableChooser<String> songChooser = new SendableChooser<>();
  private SendableChooser<String> startStop = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    addPeriodic(()->Controller.getAccessoryInstance().accessoryPeriodic(),0.020,0.010);

    // Song Chooser for Music Playing
    songChooser.setDefaultOption("Song 10", Filesystem.getDeployDirectory() + "/" + "song10.chrp");
    songChooser.addOption("Congrats", Filesystem.getDeployDirectory() + "/" + "congrats.chrp");
    songChooser.addOption("Under The Sea", Filesystem.getDeployDirectory() + "/" + "underthesea.chrp");
    songChooser.addOption("SpongeBob Opening Song", Filesystem.getDeployDirectory() + "/" + "spongebobopening.chrp");
    songChooser.addOption("Wellerman Sea Shanty", Filesystem.getDeployDirectory() + "/" + "wellerman.chrp");
    songChooser.addOption("Jeopardy", Filesystem.getDeployDirectory() + "/" + "jeopardy.chrp");
    songChooser.addOption("Song 2", Filesystem.getDeployDirectory() + "/" + "song2.chrp");
    songChooser.addOption("Song 5", Filesystem.getDeployDirectory() + "/" + "song5.chrp");
    songChooser.addOption("Eye Of The Tiger", Filesystem.getDeployDirectory() + "/" + "eyetiger.chrp");
    songChooser.addOption("He's a Pirate", Filesystem.getDeployDirectory() + "/" + "hepirate.chrp");
    songChooser.addOption("Up is Down", Filesystem.getDeployDirectory() + "/" + "updown.chrp");
    songChooser.addOption("Davy Jones' Theme", Filesystem.getDeployDirectory() + "/" + "davyjones.chrp");

    startStop.setDefaultOption("Stop", STOP_MUSIC);
    startStop.addOption("Start", START_MUSIC);

    Shuffleboard.getTab("Music")
    .add("Song Selector", songChooser)
    .withWidget(BuiltInWidgets.kComboBoxChooser);
    Shuffleboard.getTab("Music")
    .add("Music Control", startStop)
    .withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  boolean noDS = true;
  @Override
  public void robotPeriodic() {

    // Record data as long as robot is enabled (and some time after to keep recording between auto and teleop)
    m_recordTrigger.calculate(m_recording.calculate(isEnabled()));
    
    CommandScheduler.getInstance().run();
    
    changedAlly = false;
    if ((noDS || myAlliance==Alliance.Blue) && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      myAlliance = Alliance.Red;
      changedAlly = true;
      noDS = false;
    } else if ((noDS || myAlliance==Alliance.Red) && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
      myAlliance = Alliance.Blue;
      changedAlly = true;
      noDS = false;
    } else if (DriverStation.getAlliance().isPresent() == false){
      noDS = true;
    }

    if (changedAlly){
      FlightStick.m_blueAlly = (myAlliance == Alliance.Blue ? true : false);
      PoseEstimator.getInstance().reset();
      Gyro.getInstance().setGyroInit((myAlliance == Alliance.Blue ? 0 : Math.PI), 0, 0);
      PathPlanning.getInstance().calcFieldGraph();
    }

    // Synchronize the orchestra flag with the actual playing state
    if (orchestraIsPlaying != m_orchestra.isPlaying()) {
        orchestraIsPlaying = m_orchestra.isPlaying();
    }
  
  }

  String[] autoArray = {"Do Nothing"
  , "Build Your Own A(uto)dventure"
  , "FollowLine"};

  {SmartDashboard.putStringArray("Auto List", autoArray);}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  Pose2d poseTo;
  Pose2d poseFrom;
  {
    PoseEstimator.getInstance().m_field.getObject("poseTo").setPose(0,0, new Rotation2d(0));
    PoseEstimator.getInstance().m_field.getObject("poseFrom").setPose(0,0, new Rotation2d(0));
  }

  @Override
  public void autonomousInit() {

    m_orchestra.stopMusic(); // Stopping music when we switch to auto mode
    orchestraIsPlaying = false;

    switch(SmartDashboard.getString("Auto Selector", "Build Your Own A(uto)dventure")) {
      default:
      case "Build Your Own A(uto)dventure":
        m_autonomousCommand = Dashboard.getInstance().BuildYourOwnAutoCommands();
        break;
      case "FollowLine":
        Pose2d from = PoseEstimator.getInstance().m_field.getObject("poseFrom").getPose();
        Pose2d to = PoseEstimator.getInstance().m_field.getObject("poseTo").getPose();
        m_autonomousCommand = new InstantCommand(()->PathPlanning.getInstance().navigateTo(to, from));
        break;
        
        //m_autonomousCommand = Dashboard.getInstance().BuildYourOwnAutoCommands();
   }
    CommandScheduler.getInstance().schedule(m_autonomousCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    m_orchestra.stopMusic(); // Stopping music when we switch to teleop mode
    orchestraIsPlaying = false;
    DriveTrain.getInstance().m_poseQueue.clear();
    Autonomous.getInstance().stopAiming();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    DriveTrain.getInstance().m_poseQueue.clear();
    Autonomous.getInstance().stopAiming();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    selectedSong = songChooser.getSelected();
    selectedChoice = startStop.getSelected();
        if (!DriverStation.isFMSAttached() && !isTeleopEnabled() && START_MUSIC.equals(selectedChoice) && m_orchestra.isReady()) {
            if (!orchestraIsPlaying) {
              m_orchestra.playMusic(selectedSong);
              orchestraIsPlaying = true;
            }
        } else if (!DriverStation.isFMSAttached() && !isTeleopEnabled() && STOP_MUSIC.equals(selectedChoice)) {
            m_orchestra.stopMusic();
            orchestraIsPlaying = false;
        }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
