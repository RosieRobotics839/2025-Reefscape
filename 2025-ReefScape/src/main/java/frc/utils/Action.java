package frc.utils;

public class Action {
    public boolean m_state;
    
    public boolean m_enable = true;
    public Runnable onTrue;
    public Runnable onFalse;
    public Runnable onChange;
    
    public Action(){
        m_state = false;
    }

    public Action(boolean init){
        m_state = init;
    }

    public Action disable(){
        m_enable = false;
        return this;
    }

    public Action enable(){
        m_enable = true;
        return this;
    }

    public Action onTrue(Runnable action){
        onTrue = action;
        return this;
    }
    
    public Action onFalse(Runnable action){
        onFalse = action;
        return this;
    }
    
    public Action onChange(Runnable action){
        onChange = action;
        return this;
    }

    public void calculate(boolean val){
        if (m_state != val){
            if (m_enable && onChange != null){
                onChange.run();
            }
            if (m_enable && val && onTrue != null){
                onTrue.run();
            }
            if (m_enable && !val && onFalse != null){
                onFalse.run();
            }
            m_state = val;
        }
    }
}