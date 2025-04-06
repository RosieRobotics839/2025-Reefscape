// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.GyroConstants;
import frc.utils.Hysteresis;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Gyro extends SubsystemBase {
  
  private double lastSimYawRad = 0;
  private double simYaw = 0;

  private static PigeonIMU pidgey1;
  private static Pigeon2 pidgey2;
  private static Pigeon2SimState simState;

  public boolean m_enableTipDetection = true;

  private static Gyro gyro = new Gyro();
  
  public static Gyro getInstance(){
    return gyro;
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Gyro");

  Debouncer m_warningTimer = new Debouncer(10,DebounceType.kRising);

  public void setDefaultHeading() {
    initypr[0] = ypr[0];
  }

  // Constructor Function on init, set gyro yaw to zero.
  public Gyro() {
    
    if (GyroConstants.kEnabled && !Robot.isSimulation()){
      if (GyroConstants.kIsPigeon2){
        simState = new Pigeon2SimState(pidgey2);
        pidgey2.getConfigurator().apply(new Pigeon2Configuration());
        pidgey2.getYaw().setUpdateFrequency(100);
        pidgey2.setYaw(0);
      } else {
        pidgey1 = new PigeonIMU(GyroConstants.kCANID);
        pidgey1.setYaw(0);
      }
    }
  } 

  double [] initypr = {0,0,0};
  double [] lastypr = {0,0,0};
  double [] ypr = {0,0,0};
  
  public boolean init = true;
  NTBoolean nt_init = new NTBoolean(false,table,"init",val->init=val); {nt_init.resetOnRecv = true;}

  Hysteresis m_isTipped = new Hysteresis().withThreshold(GyroConstants.kTippingAngle).withHysteresis(GyroConstants.kTippingHysteresis);

  final DoublePublisher nt_yaw = table.getDoubleTopic("yaw").publish();
  final DoublePublisher nt_pitch = table.getDoubleTopic("pitch").publish();
  final DoublePublisher nt_roll = table.getDoubleTopic("roll").publish();
  final DoublePublisher nt_offset = table.getDoubleTopic("offset").publish();
  final BooleanPublisher nt_status = table.getBooleanTopic("status").publish();
  final BooleanPublisher nt_isTipped = table.getBooleanTopic("isTipped").publish();
  final DoublePublisher nt_tipHeading = table.getDoubleTopic("tipHeading").publish();
  final DoublePublisher nt_gvx = table.getDoubleTopic("gvx").publish();
  final DoublePublisher nt_gvy = table.getDoubleTopic("gvy").publish();
  final DoublePublisher nt_gvz = table.getDoubleTopic("gvz").publish();

  final NTDouble nt_simOffset = new NTDouble(0.0,table,"CAUTION/offset",val->initypr[0]=initypr[0]+val); {nt_simOffset.resetOnRecv=true;}
  final DoublePublisher nt_simYaw = table.getDoubleTopic("sim/yaw").publish();

  double m_gyroOffset = 0;
  double m_lastReset = 5;

  @Override
  public void periodic() {

    if (m_warningTimer.calculate(!GyroConstants.kEnabled && (DriverStation.isFMSAttached() || DriverStation.isTestEnabled()))){
      m_warningTimer.calculate(false);
      System.err.println("ERROR: Gyro was not detected on the CAN bus and is being SIMULATED!");
    }

    if (!GyroConstants.kEnabled || Robot.isSimulation()) {
      // In simulation, just use the pose estimator's rotation
      if (Robot.isSimulation()) {
        ypr[0] = PoseEstimator.getInstance().m_sim_actualPose.getRotation().getRadians();
        nt_yaw.set(Units.radiansToDegrees(ypr[0]));
        nt_status.set(false);
      }
      return;
    }

    // If there is a hardware fault, attempt to reset the gyro every 5 seconds
    m_lastReset = Math.max(0, m_lastReset-0.02);
    if (pidgey2 != null && pidgey2.getFault_Hardware().getValue() && m_lastReset == 0){
      pidgey2.reset();
      m_lastReset = 5;
      initypr = new double []{0,0,0};
    }

    // If robot is disabled allow calibration of the gyro by user button press or over network tables
    if (!RobotController.isSysActive() && (init || RobotController.getUserButton())){
      double m_calibrateYaw = 0;
      if (!FlightStick.m_blueAlly){
        m_calibrateYaw = Math.PI; // Red Alliance calibrates to 180 degrees
      }
      setGyroInit(m_calibrateYaw,0,0);
    }
    
    init = false;
    
    // Get new ypr values in degrees
    double [] newypr = getypr();
    getTippingAngle();
    m_isTipped.calculate(m_enableTipDetection ? VectorUtils.SRSS(VectorUtils.angleDifference(0,getPitch()),VectorUtils.angleDifference(0,getRoll())) : 0);

    // Convert the continuous values offset by the init or vision correction to radians, this outputs values between 0 and 2*pi
    ypr[0] = (((newypr[0]+initypr[0]) % 360)*(Math.PI)/180.0 + 2*Math.PI) % (2*Math.PI);
    ypr[1] = (((newypr[1]+initypr[1]) % 360)*(Math.PI)/180.0 + 2*Math.PI) % (2*Math.PI);
    ypr[2] = (((newypr[2]+initypr[2]) % 360)*(Math.PI)/180.0 + 2*Math.PI) % (2*Math.PI);

    // Vision correction using april tags, if data is valid
    if (PoseEstimator.getInstance().m_visionIsValid){
      initypr[0] += 180.0/Math.PI * Math.max(Math.min(VectorUtils.angleDifference(PoseEstimator.getInstance().m_visionTheta.getRadians(),ypr[0]), GyroConstants.kVisionCorrectionMaxRate * 0.020), -GyroConstants.kVisionCorrectionMaxRate * 0.020);
    }

    nt_isTipped.set(m_isTipped.get());
    nt_status.set(getStatus());
    nt_yaw.set(Units.radiansToDegrees(ypr[0]));
    nt_pitch.set(Units.radiansToDegrees(ypr[1]));
    nt_roll.set(Units.radiansToDegrees(ypr[2]));
    nt_offset.set(initypr[0]); // is in degrees
    nt_simYaw.set(simYaw); // is in degrees
  }

  public double [] getypr(){
    if (!GyroConstants.kEnabled || Robot.isSimulation()){
      return new double[]{0, 0, 0};
    }

    // read sensor data from Pigeon2
    if (Robot.isSimulation()){
      double simyawrad = PoseEstimator.getInstance().m_sim_actualPose.getRotation().getRadians();
      if (simState != null){
        simState.addYaw(Units.radiansToDegrees(VectorUtils.angleDifference(simyawrad,lastSimYawRad)));
      } else {
        pidgey1.setYaw(Units.radiansToDegrees(simyawrad));
      }
    }

    double yaw = ((pidgey2 != null ? pidgey2.getYaw().getValueAsDouble() : pidgey1.getYaw()) % 360 + 360) % 360;
    double pitch = ((pidgey2 != null ? pidgey2.getRoll().getValueAsDouble() : pidgey1.getRoll()) % 360 + 360) % 360;
    double roll = ((pidgey2 != null ? pidgey2.getPitch().getValueAsDouble() : pidgey1.getPitch()) % 360 + 360) % 360;

    return new double[]{yaw, pitch, roll};
  }

  public double getYaw(){
    return ypr[0];

  }
  public double getPitch(){
    return ypr[1];
  }

  public double getRoll(){
    return ypr[2];
  }

  /**
   * Returns the direction that the robot is tipping.
   * The returned angle is relative to the robot
   * @return angle from forward in radians
   */
  public double getTippingAngle(){
    double gvx = -pidgey.getGravityVectorY().getValueAsDouble();
    double gvy = pidgey.getGravityVectorX().getValueAsDouble();
    nt_gvx.set(gvx);
    nt_gvy.set(gvy);
    if (Math.abs(gvx) <= 1e-6 && Math.abs(gvy) <= 1e-6){
      return 0;
    }
    double tipHeading = new Translation2d(gvx,gvy).getAngle().getRadians();
    nt_tipHeading.set(Units.radiansToDegrees(tipHeading));
    return tipHeading;
  }

  public void resetPrimarySensor() {
    if (pidgey2 != null){
      pidgey2.reset();
    }
  }

  public boolean isTipping(){
    return m_isTipped.get();
  }

  public void setGyroInit(double _y, double _p, double _r){
    double [] newypr = (Robot.isReal() ? getypr() : new double[]{PoseEstimator.getInstance().m_sim_actualPose.getRotation().getDegrees(),0,0});
    initypr[0] =  Units.radiansToDegrees(_y)-newypr[0];
    initypr[1] =  Units.radiansToDegrees(_p)-newypr[1];
    initypr[2] =  Units.radiansToDegrees(_r)-newypr[2];
  }

  // used in simulation
  public void setypr(double y, double p, double r){
    this.ypr[0] = (y % 360 + 360) % 360;
    this.ypr[1] = p;
    this.ypr[2] = r;
  }

  public boolean getStatus() {
    if (Robot.isSimulation()) {
      return false;  // Always return false in simulation
    }
    boolean GyroGood = false;
    if (GyroConstants.kEnabled){
        // getFault_Hardware() returns False if the hardware is good
        GyroGood = !(pidgey2 != null && pidgey2.getFault_Hardware().getValue());
    }
   
    return GyroGood;
  }
  
}