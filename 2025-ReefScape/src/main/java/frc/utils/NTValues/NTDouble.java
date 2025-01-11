package frc.utils.NTValues;

import java.util.EnumSet;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;

public class NTDouble {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Constants");

    private DoubleSubscriber subscriber;

    private DoublePublisher publisher;

    public boolean resetOnRecv = false;

    private boolean m_ignore = false;

    public static double create(double defaultValue, String name, DoubleConsumer lambda){
        NTDouble instance = new NTDouble(defaultValue, table, name, lambda);
        return instance.get();
    }

    public static double create(double defaultValue, NetworkTable _table, String name, DoubleConsumer lambda){
        NTDouble instance = new NTDouble(defaultValue, _table, name, lambda);
        return instance.get();
    }

    public NTDouble(Double defaultValue, NetworkTable _table, String name, DoubleConsumer lambda){
        subscriber = _table.getDoubleTopic(name).subscribe(defaultValue);
        publisher = _table.getDoubleTopic(name).publish();
        publisher.set(defaultValue);
        
        // add a listener to only value changes on the Y subscriber
        _table.addListener(
            name,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table,key,event)-> {
                if (!m_ignore){
                    lambda.accept(event.valueData.value.getDouble());
                    if (resetOnRecv){
                        set(defaultValue);
                    }
                } else {
                  m_ignore = false;
                }
            });
    }

    public void set(double val){
        m_ignore = true;
        publisher.set(val);
    }

    public TimestampedDouble getAtomic(){
        return subscriber.getAtomic();
    }

    public Double get(){
        return subscriber.get();
    }
}