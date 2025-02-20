package frc.utils.NTValues;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

public class NTDoubleArray {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Constants");

    private DoubleArraySubscriber subscriber;

    private DoubleArrayPublisher publisher;

    public boolean resetOnRecv = false;

    public static double [] create(double [] defaultValue, String name, Consumer<double []> lambda){
        NTDoubleArray instance = new NTDoubleArray(defaultValue, table, name, lambda);
        return instance.get();
    }

    public static double [] create(double [] defaultValue, NetworkTable _table, String name, Consumer<double []> lambda){
        NTDoubleArray instance = new NTDoubleArray(defaultValue, _table, name, lambda);
        return instance.get();
    }

    public NTDoubleArray(double [] defaultValue, NetworkTable _table, String name, Consumer<double[]> lambda){
        subscriber = _table.getDoubleArrayTopic(name).subscribe(defaultValue);
        publisher = _table.getDoubleArrayTopic(name).publish();
        publisher.set(defaultValue);

        // add a listener to only value changes on the Y subscriber
        _table.addListener(
            name,
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
            (t,k,e)-> {
                lambda.accept(e.valueData.value.getDoubleArray());
                if (resetOnRecv){
                    set(defaultValue);
                }
            }
        );
    }

    public void set(double [] val){
        publisher.set(val);
    }

    public TimestampedDoubleArray getAtomic(){
        return subscriber.getAtomic();
    }

    public double [] get(){
        return subscriber.get();
    }
}