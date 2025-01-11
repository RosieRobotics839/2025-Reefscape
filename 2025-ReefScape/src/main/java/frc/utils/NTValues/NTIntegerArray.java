package frc.utils.NTValues;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedIntegerArray;

public class NTIntegerArray {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Constants");

    private IntegerArraySubscriber subscriber;

    private IntegerArrayPublisher publisher;
    
    public boolean resetOnRecv = false;

    private boolean m_ignore = false;

    public static long [] create(long [] defaultValue, String name, Consumer<long []> lambda){
        NTIntegerArray instance = new NTIntegerArray(defaultValue, table, name, lambda);
        return instance.get();
    }

    public static long [] create(long [] defaultValue, NetworkTable _table, String name, Consumer<long []> lambda){
        NTIntegerArray instance = new NTIntegerArray(defaultValue, _table, name, lambda);
        return instance.get();
    }

    public NTIntegerArray(long [] defaultValue, NetworkTable _table, String name, Consumer<long[]> lambda){
        subscriber = _table.getIntegerArrayTopic(name).subscribe(defaultValue);
        publisher = _table.getIntegerArrayTopic(name).publish();
        publisher.set(defaultValue);

        // add a listener to only value changes on the Y subscriber
        _table.addListener(
            name,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (t,k,e)-> {
                if (!m_ignore){
                    lambda.accept(e.valueData.value.getIntegerArray());
                    if (resetOnRecv){
                        publisher.set(defaultValue);
                    }
                } else {
                    m_ignore = false;
                }
            });
    }

    public void set(long [] val){
        m_ignore = true;
        publisher.set(val);
    }

    public TimestampedIntegerArray getAtomic(){
        return subscriber.getAtomic();
    }

    public long [] get(){
        return subscriber.get();
    }
}