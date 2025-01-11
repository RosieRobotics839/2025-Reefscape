package frc.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class CalibrationMap extends InterpolatingDoubleTreeMap {

  private double m_xmin = Double.POSITIVE_INFINITY; 
  private double m_xmax = Double.NEGATIVE_INFINITY; 
  private double m_ymin = Double.POSITIVE_INFINITY; 
  private double m_ymax = Double.NEGATIVE_INFINITY; 
  public double xmin(){return m_xmin;};
  public double xmax(){return m_xmax;};
  public double ymin(){return m_ymin;};
  public double ymax(){return m_ymax;};
  public CalibrationMap(double [] x, double [] y){
    for (int i=0; i < Math.min(x.length,y.length); i++){
      this.put(x[i],y[i]);
      m_xmin = (x[i] < m_xmin ? x[i] : m_xmin);
      m_xmax = (x[i] > m_xmax ? x[i] : m_xmax);
      m_ymin = (y[i] < m_ymin ? y[i] : m_ymin);
      m_ymax = (y[i] > m_ymax ? y[i] : m_ymax);
    }
  }
}