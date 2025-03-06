// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.stream.LongStream;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Controller.AccessoryButtons;

public class LED extends SubsystemBase {

  private static final LED led = new LED();
  public static LED getInstance() {
    return led;
  }

  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  
  private boolean testBool = true; 
  private double lastFlash = Timer.getFPGATimestamp(); 

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

  boolean m_systemhealthy;

  @Override
  public void periodic() {

    m_systemhealthy = true;
    setPixels(LEDConstants.kUnhealthyColor, LEDConstants.kAllLEDs);
    
    /* Arm */
    
    Arm _arm = Arm.getInstance();
    // Test Arm Motor Temperature
    testBool = _arm.m_motor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool){
      m_systemhealthy = false;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kArmLEDs);
    }

    // Arm Setup Detection
    if (!_arm.m_setupDone){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupFailColor, LEDConstants.kArmLEDs));
    }

    /* Elevator */
    
    Elevator _elevator = Elevator.getInstance();
    // Test Elevator Motor Temperature
    testBool = _elevator.m_EleMotor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool) {
      m_systemhealthy = false;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kElevatorLEDs);
    }

    // Elevator setup detection
    if (!_elevator.setupElevator || (DriverStation.isEnabled() && !_elevator.m_isCalibrated.get())){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupFailColor, LEDConstants.kElevatorLEDs));
    }

    /* End Effector */
    
    EndEffector _effector = EndEffector.getInstance();
    AccessoryButtons _controller = Controller.getAccessoryButtonsInstance();
    // Test Effector Motor Temperature
    testBool = _effector.m_motor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool){
      m_systemhealthy = false;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kEffectorLEDs);
    } 

    /* Elevator */
    
    Climber _climber = Climber.getInstance();
    // Test Climber Motor Temperature
    testBool = _climber.m_motor.getMotorTemperature() > LEDConstants.kMaxMotorTemp;
    if (testBool){
      m_systemhealthy = false;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kClimberLEDs);
    }

    // Climber setup detection
    if (!_climber.m_setupDone){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupFailColor, LEDConstants.kClimberLEDs));
    }

    /* DriveTrain */
    
    // Test Motor Temperatures
    testBool = false;
    DriveTrain.forEachSwerveModule((s)->testBool = testBool || s.m_motorDrive.getMotorTemperature() > LEDConstants.kMaxMotorTemp || s.m_motorSteer.getMotorTemperature() > LEDConstants.kMaxMotorTemp);
    if (testBool){
      m_systemhealthy = false;
      setPixels(LEDConstants.kMotorTempColor, LEDConstants.kSwerveLEDs);
    }

    // Swerve Setup Detection 
    if (!DriveTrain.getInstance().m_motorSetupDone){
      // Swerve motors not configured
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupFailColor, LEDConstants.kSwerveLEDs));
    }

    // Test Cameras Connected
    if (!Vision.cam1.isConnected() || !Vision.cam2.isConnected()){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupFailColor, LEDConstants.kPhotonVisionLEDs));
    }

    // Check Gyro Status
    if (Gyro.getInstance().getStatus() == false){
      m_systemhealthy = false;
      flash(()->setPixels(LEDConstants.kSetupFailColor, LEDConstants.kGyroLEDs));
    }

    if (m_systemhealthy){
      if (DriverStation.isDisabled()){
        setAltColors(LEDConstants.kHealthyColor1, LEDConstants.kHealthyColor2, LEDConstants.kAllLEDs);
      } else {
        if (_controller.isAlgaeSelected){
          setPixels(LEDConstants.kAlgaeColor, LEDConstants.kAllLEDs);
        } else {
          setPixels(LEDConstants.kCoralColor, LEDConstants.kAllLEDs);
        }
      }
      
      // Checking to see if we have a game piece
      if (EndEffector.getInstance().hasGamePiece()){
        flash(()->setPixels(LEDConstants.kActivityColor, LEDConstants.kEffectorLEDs));
      }

      // Check PoseEstimator Updating
      PoseEstimator pe = PoseEstimator.getInstance();
      if (pe.m_gyroResidual > LEDConstants.kPoseResidualRot || 
          pe.m_visionPoseResidual.getTranslation().getNorm() > LEDConstants.kPoseResidualDist ||
          pe.m_visionPoseResidual.getRotation().getRadians() > LEDConstants.kPoseResidualRot){
          setPixels(LEDConstants.kActivityColor, LEDConstants.kPoseEstimatorLEDs);
      }
      
      // Photonvision Detected AprilTag
      if (Vision.getInstance().m_numTargets > 0){
        setPixels(LEDConstants.kActivityColor, LEDConstants.kPhotonVisionLEDs);
      }
    }
    sendData();

  }
}