// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

/** Add your docs here. */
public class FirstOrderLag  {
    public double value, tau;
    private double m_period;

    public FirstOrderLag(double timeConstant, double initialValue,  double period){
        value = initialValue;
        m_period = period;
        tau = timeConstant;
    }
    
    public double calculate(double inValue){
        double dynamicGain = Math.max(0,Math.min(1,m_period/(Math.min(1,tau))));
        value = value + dynamicGain*(inValue-value); 
        return value;
    }
}
