package frc.utils.CANSparkMax;

import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;

public class MyCANSparkMax extends com.revrobotics.spark.SparkMax{
    
    public MyCANSparkMax(int deviceId, MotorType type){
        super( deviceId,  type);
    }
    
    public REVLibError setInverted2(boolean isInverted) {
        throwIfClosed();
        return REVLibError.fromInt(
            CANSparkJNI.c_Spark_SetInverted(sparkHandle, isInverted));
    }

}
