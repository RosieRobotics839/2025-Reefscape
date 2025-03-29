// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.stream.LongStream;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/LED");

  private static final LED led = new LED();
  public static LED getInstance() {
    return led;
  }

  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  
  private boolean testBool = true;
  public boolean m_driveTrainMotorsTempHigh = false;
  public boolean m_elevatorMotorTempHigh = false;
  public boolean m_armMotorTempHigh = false;
  public boolean m_effectorMotorTempHigh = false;
  public boolean m_funnelMotorTempHigh = false;
  public boolean m_climberMotorTempHigh = false;
  public int TestTimeRemaining = 150; // 2 and a half minutes;

  BooleanPublisher nt_driveTrainMotorsTempHigh = table.getBooleanTopic("driveTrainMotorsTempHigh").publish();
  BooleanPublisher nt_elevatorMotorTempHigh = table.getBooleanTopic("elevatorMotorTempHigh").publish();
  BooleanPublisher nt_armMotorTempHigh = table.getBooleanTopic("armMotorTempHigh").publish();
  BooleanPublisher nt_effectorMotorTempHigh = table.getBooleanTopic("effectorMotorTempHigh").publish();
  BooleanPublisher nt_funnelMotorTempHigh = table.getBooleanTopic("funnelMotorTempHigh").publish();
  BooleanPublisher nt_climberMotorTempHigh = table.getBooleanTopic("climberMotorTempHigh").publish();
  DoublePublisher nt_remainingMatchTime = table.getDoubleTopic("remainingMatchTime").publish();

  private double lastFlash = Timer.getFPGATimestamp(); 
  double FMSTimeRemaining = Timer.getMatchTime();

  double lastUpdate = 0;

  /** Creates a new LEDs. */
  public LED() {
        // PWM port /

    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(LEDConstants.kLEDPortPWM);

   
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kNumberLEDs);
    m_led.setLength(LEDConstants.kNumberLEDs);

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setPixels(int [] c, long ... pixels){
    for (int i=0; i<pixels.length; i++){
      if (c == null || pixels == null) return;
      // Sets the LEDs to the set RGB value
      m_ledBuffer.setRGB((int)pixels[i], (int)(c[0]*LEDConstants.kBrightness), (int)(c[1]*LEDConstants.kBrightness), (int)(c[2]*LEDConstants.kBrightness));
    }
  }

  public void flash(Runnable lambda){
    if (Timer.getFPGATimestamp()-lastFlash < LEDConstants.kFlashTime){
      lambda.run();
    } else if (Timer.getFPGATimestamp()-lastFlash > LEDConstants.kFlashTime*2){
      lastFlash = Timer.getFPGATimestamp();
    }
  }

  public void sendData(){
    m_led.setData(m_ledBuffer);
  }

  //For setting alternating colors
  public void setAltColors(int c[], int c2[], long ... pixels){

    if (c == null || c2 == null || pixels == null) return;
    long [] pixels1 = LongStream.iterate(0, i -> i + 2).limit(pixels.length/2).toArray(); 
    long [] pixels2 = LongStream.iterate(1, i -> i + 2).limit(pixels.length-pixels1.length).toArray(); 

    setPixels(c, pixels1);
    setPixels(c2, pixels2);
  }

  private double lastTime = 0;

  public void updateMatchTimer() {
      double currentTime = Timer.getFPGATimestamp();
      
      if (DriverStation.isEnabled()) {
          if (currentTime - lastTime >= 1.0) { // Decrease once per second
              if (TestTimeRemaining > 0) {
                  TestTimeRemaining--;
              }
              lastTime = currentTime;
          }
          
          if (TestTimeRemaining < 30 && TestTimeRemaining > 25) {
              flash(() -> setPixels(LEDConstants.kClimbColor, LEDConstants.kAllLEDs));
          }
      }
  }

  boolean m_systemhealthy;

  @Override
  public void periodic() {

    m_systemhealthy = true;
    
    setPixels(LEDConstants.kUnhealthyColor, LEDConstants.kAllLEDs);

    updateMatchTimer();

    /* Arm */
    
    Arm _arm = Arm.getInstance();
    // Test Arm Motor Temperature
    testBool = _arm.m_motor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool){
      m_systemhealthy = false;
      m_armMotorTempHigh = true;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kArmLEDs);
    } else {
      m_armMotorTempHigh = false;
    }

    // Arm Setup Detection
    if (!_arm.m_setupDone){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupMovementFailColor, LEDConstants.kArmLEDs));
    }

    /* Elevator */
    
    Elevator _elevator = Elevator.getInstance();
    // Test Elevator Motor Temperature
    testBool = _elevator.m_EleMotor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool) {
      m_systemhealthy = false;
      m_elevatorMotorTempHigh = true;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kElevatorLEDs);
    } else {
      m_elevatorMotorTempHigh = false;
    }

    // Elevator setup detection
    if (!_elevator.m_EleMotor.isSetupDone() || (DriverStation.isEnabled() && !_elevator.m_isCalibrated.get())){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupMovementFailColor, LEDConstants.kElevatorLEDs));
    }

    /* End Effector */
    
    EndEffector _effector = EndEffector.getInstance();
    // Test Effector Motor Temperature
    testBool = _effector.m_motor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool){
      m_systemhealthy = false;
      m_effectorMotorTempHigh = true;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kEffectorLEDs);
    } else {
      m_effectorMotorTempHigh = false;
    }

    /* Elevator */
    
    Climber _climber = Climber.getInstance();
    // Test Climber Motor Temperature
    testBool = _climber.m_motor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool){
      m_systemhealthy = false;
      m_climberMotorTempHigh = true;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kClimberLEDs);
    } else {
      m_climberMotorTempHigh = false;
    }

    // Climber setup detection
    if (!_climber.m_motor.isSetupDone()){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupMovementFailColor, LEDConstants.kClimberLEDs));
    }

    /* DriveTrain */
    
    // Test Motor Temperatures
    testBool = false;
    DriveTrain.forEachSwerveModule((s)->testBool = testBool || s.m_motorDrive.getMotorTemperature() > LEDConstants.kMaxMotorTemp || s.m_motorSteer.getMotorTemperature() > LEDConstants.kMaxMotorTemp);
    if (testBool){
      m_systemhealthy = false;
      m_driveTrainMotorsTempHigh = true;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kSwerveLEDs);
    } else {
      m_driveTrainMotorsTempHigh = false;
    }

    // Swerve Setup Detection 
    if (!DriveTrain.getInstance().m_motorSetupDone){
      // Swerve motors not configured
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupMovementFailColor, LEDConstants.kSwerveLEDs));
    }

    // Test Cameras Connected
    if (!Vision.cam1.isConnected() || !Vision.cam2.isConnected()){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupAwarenessFailColor, LEDConstants.kPhotonVisionLEDs));
    }

    // Check Gyro Status
    if (Gyro.getInstance().getStatus() == false){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupAwarenessFailColor, LEDConstants.kGyroLEDs));
    }

    if (m_systemhealthy){
      setAltColors(LEDConstants.kHealthyColor1, LEDConstants.kHealthyColor2, LEDConstants.kAllLEDs);
	  }
      
      // Checking to see if we have a game piece
      if (EndEffector.getInstance().hasGamePiece()){
        flash(()->setPixels(LEDConstants.kActivityColor, LEDConstants.kAllLEDs));
      }

      // Check PoseEstimator Updating
      PoseEstimator pe = PoseEstimator.getInstance();
      if (pe.m_gyroResidual > LEDConstants.kPoseResidualRot || 
          pe.m_visionPoseResidual.getTranslation().getNorm() > LEDConstants.kPoseResidualDist ||
          pe.m_visionPoseResidual.getRotation().getRadians() > LEDConstants.kPoseResidualRot){
          setPixels(LEDConstants.kActivityColor, LEDConstants.kAllLEDs);
      }
      
      // Photonvision Detected AprilTag
      if (Vision.getInstance().m_numTargets > 0){
        //setPixels(LEDConstants.kActivityColor, LEDConstants.kAllLEDs);
      }

      nt_driveTrainMotorsTempHigh.set(m_driveTrainMotorsTempHigh);
      nt_elevatorMotorTempHigh.set(m_elevatorMotorTempHigh);
      nt_armMotorTempHigh.set(m_armMotorTempHigh);
      nt_effectorMotorTempHigh.set(m_effectorMotorTempHigh);
      nt_funnelMotorTempHigh.set(m_funnelMotorTempHigh);
      nt_climberMotorTempHigh.set(m_climberMotorTempHigh);
      nt_remainingMatchTime.set(TestTimeRemaining /*FMSTimeRemaining*/);

      sendData();
  }
}