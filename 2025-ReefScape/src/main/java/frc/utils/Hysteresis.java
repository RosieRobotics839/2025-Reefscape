package frc.utils;

public class Hysteresis {
    private boolean m_state;
    private double m_hysteresis;
    private double m_threshold;

    private Runnable m_onTrue = null;
    private Runnable m_onFalse = null;

    public Hysteresis(){
    }

    public boolean calculate(double input){
        if (m_state){
            if (input <= m_threshold - m_hysteresis){
                m_state = false;
                if (m_onFalse != null) m_onFalse.run();
            }
        } else { 
            if (input >= m_threshold + m_hysteresis){
                m_state = true;
                if (m_onTrue != null) m_onTrue.run();
            }
        }
        return m_state;
    }
    public void set(boolean state){
        m_state = state;
    }
    public Hysteresis withHysteresis(double hysteresis){
        m_hysteresis = hysteresis;
        return this;
    }
    public Hysteresis withThreshold(double threshold){
        m_threshold = threshold;
        return this;
    }
    public Hysteresis onTrue(Runnable action){
        m_onTrue = action;
        return this;
    }
    public Hysteresis onFalse(Runnable action){
        m_onFalse = action;
        return this;
    }

}
