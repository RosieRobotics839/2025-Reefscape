package frc.utils.NTValues;

import java.util.EnumSet;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedBoolean;
import edu.wpi.first.util.function.BooleanConsumer;

public class NTBoolean {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Constants");

    private BooleanSubscriber subscriber;

    private BooleanPublisher publisher;
    
    public boolean resetOnRecv = false;

    public static Boolean create(Boolean defaultValue, String name, BooleanConsumer lambda){
        NTBoolean instance = new NTBoolean(defaultValue, table, name, lambda);
        return instance.get();
    }

    public static Boolean create(Boolean defaultValue, NetworkTable _table, String name, BooleanConsumer lambda){
        NTBoolean instance = new NTBoolean(defaultValue, _table, name, lambda);
        return instance.get();
    }

    public NTBoolean(Boolean defaultValue, NetworkTable _table, String name, BooleanConsumer lambda){
        subscriber = _table.getBooleanTopic(name).subscribe(defaultValue);
        publisher = _table.getBooleanTopic(name).publish();
        publisher.set(defaultValue);
        
        // add a listener to only value changes on the Y subscriber
        _table.addListener(
            name,
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
            (table,key,event)-> {
                if (lambda != null){
                    lambda.accept(event.valueData.value.getBoolean());
                }
                if (resetOnRecv){
                    set(defaultValue);
                }
            }
        );
    }

    public void set(Boolean val){
        publisher.set(val);
    }

    public TimestampedBoolean getAtomic(){
        return subscriber.getAtomic();
    }

    public Boolean get(){
        return subscriber.get();
    }
}