package frc.utils.NTValues;

import java.util.EnumSet;
import java.util.function.IntConsumer;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedInteger;

public class NTInteger {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Constants");

    private IntegerSubscriber subscriber;

    private IntegerPublisher publisher;
    
    public boolean resetOnRecv = false;

    private boolean m_ignore = false;

    public static Integer create(Integer defaultValue, String name, IntConsumer lambda){
        NTInteger instance = new NTInteger(defaultValue, table, name, lambda);
        return instance.get();
    }

    public static Integer create(Integer defaultValue, NetworkTable _table, String name, IntConsumer lambda){
        NTInteger instance = new NTInteger(defaultValue, _table, name, lambda);
        return instance.get();
    }

    public NTInteger(Integer defaultValue, NetworkTable _table, String name, IntConsumer lambda){
        subscriber = _table.getIntegerTopic(name).subscribe(defaultValue);
        publisher = _table.getIntegerTopic(name).publish();
        publisher.set(defaultValue);
        
        // add a listener to only value changes on the Y subscriber
        _table.addListener(
            name,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table,key,event)-> {
                if (!m_ignore){
                    lambda.accept((int)event.valueData.value.getInteger());
                    if (resetOnRecv){
                        publisher.set(defaultValue);
                    }
                } else {
                    m_ignore = false;
                }
            });
    }

    public void set(Integer val){
        m_ignore = true;
        publisher.set(val);
    }

    public TimestampedInteger getAtomic(){
        return subscriber.getAtomic();
    }

    public Integer get(){
        return (int) subscriber.get();
    }
}