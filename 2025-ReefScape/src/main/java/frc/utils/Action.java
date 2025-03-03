package frc.utils;

public class Action {
    public boolean m_state;
    
    public Runnable onTrue;
    public Runnable onFalse;
    public Runnable onChange;
    
    public Action(){
        m_state = false;
    }

    public Action(boolean init){
        m_state = init;
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
            if (onChange != null){
                onChange.run();
            }
            if (val && onTrue != null){
                onTrue.run();
            }
            if (!val && onFalse != null){
                onFalse.run();
            }
            m_state = val;
        }
    }
}