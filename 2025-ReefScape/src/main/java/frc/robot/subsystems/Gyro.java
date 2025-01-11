// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.GyroConstants;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Gyro extends SubsystemBase {
  
  private double lastSimYawRad = 0;
  private double simYaw = 0;

  private static Pigeon2 pidgey = new Pigeon2(GyroConstants.kCANID_Pigeon, "rio");

  private static Gyro gyro = new Gyro(); 
  
  public static Gyro getInstance(){
    return gyro;
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Gyro");

  public void setDefaultHeading() {
    initypr[0] = ypr[0];
  }

  // Constructor Function on init, set gyro yaw to zero.
  public Gyro() {
    pidgey.getConfigurator().apply(new Pigeon2Configuration());
    pidgey.getYaw().setUpdateFrequency(100);
    pidgey.setYaw(0);
  } 

  double [] initypr = {0,0,0};
  double [] lastypr = {0,0,0};
  double [] ypr = {0,0,0};
  
  public boolean init = true;
  NTBoolean nt_init = new NTBoolean(false,table,"init",val->init=val); {nt_init.resetOnRecv = true;}

  final DoublePublisher nt_yaw = table.getDoubleTopic("yaw").publish();
  final DoublePublisher nt_pitch = table.getDoubleTopic("pitch").publish();
  final DoublePublisher nt_roll = table.getDoubleTopic("roll").publish();
  final DoublePublisher nt_offset = table.getDoubleTopic("offset").publish();
  final BooleanPublisher nt_status = table.getBooleanTopic("status").publish();

  final NTDouble nt_simOffset = new NTDouble(0.0,table,"CAUTION/offset",val->initypr[0]=initypr[0]+val); {nt_simOffset.resetOnRecv=true;}
  final DoublePublisher nt_simYaw = table.getDoubleTopic("sim/yaw").publish();

  double m_gyroOffset = 0;
  double m_lastReset = 5;

  @Override
  public void periodic() {
    // If there is a hardware fault, attempt to reset the gyro every 5 seconds
    m_lastReset = Math.max(0, m_lastReset-0.02);
    if (pidgey.getFault_Hardware().getValue() && m_lastReset == 0){
      pidgey.reset();
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

    // Convert the continuous values offset by the init or vision correction to radians, this outputs values between 0 and 2*pi
    ypr[0] = (((newypr[0]+initypr[0]) % 360)*(Math.PI)/180.0 + 2*Math.PI) % (2*Math.PI);
    ypr[1] = (((newypr[1]+initypr[1]) % 360)*(Math.PI)/180.0 + 2*Math.PI) % (2*Math.PI);
    ypr[2] = (((newypr[2]+initypr[2]) % 360)*(Math.PI)/180.0 + 2*Math.PI) % (2*Math.PI);

    // Vision correction using april tags, if data is valid
    if (PoseEstimator.getInstance().m_visionIsValid){
      initypr[0] += 180.0/Math.PI * Math.max(Math.min(VectorUtils.angleDifference(PoseEstimator.getInstance().m_visionTheta.getRadians(),ypr[0]), GyroConstants.kVisionCorrectionMaxRate * 0.020), -GyroConstants.kVisionCorrectionMaxRate * 0.020);
    }

    nt_status.set(getStatus());
    nt_yaw.set(Units.radiansToDegrees(ypr[0]));
    nt_pitch.set(Units.radiansToDegrees(ypr[1]));
    nt_roll.set(Units.radiansToDegrees(ypr[2]));
    nt_offset.set(initypr[0]); // is in degrees
    nt_simYaw.set(simYaw); // is in degrees
  }

  public double [] getypr(){
    // read sensor data from Pigeon2
    if (Robot.isSimulation()){
      double simyawrad = PoseEstimator.getInstance().m_sim_actualPose.getRotation().getRadians();
      simYaw += Units.radiansToDegrees(VectorUtils.angleDifference(simyawrad,lastSimYawRad));
      lastSimYawRad = simyawrad;
      pidgey.setYaw(simYaw);
    }
    double yaw = (pidgey.getYaw().getValue() % 360 + 360) % 360;
    // we're not using pitch and roll, don't bother requesting them over the CAN bus.
    // double pitch = (pidgey.getPitch().getValue();
    // double roll = (pidgey.getRoll().getValue(); 
    double pitch = 0;
    double roll = 0;

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

  public void resetPrimarySensor() {
    pidgey.reset();
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
    // getFault_Hardware() returns False if the hardware is good
    boolean GyroGood = !pidgey.getFault_Hardware().getValue();
   
    return GyroGood;
  }
  
}