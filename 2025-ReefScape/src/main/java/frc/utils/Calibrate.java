package frc.utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;

public class Calibrate extends CalibrationMap {
    public Motor motor;
    public BooleanSupplier ready;
    public DoubleSupplier inSupplier;
    public DoublePredicate action;
    public Command onFinished;
    public Command calibrationCmd;

    static NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/Test");
    NTDouble nt_x, nt_y;
    NTBoolean nt_ready, nt_action, nt_done;

    public Calibrate(String name, double [] calibration_x, double [] calibration_y, BooleanSupplier ready, DoubleSupplier input, DoublePredicate action, Command onFinished){
        super(calibration_x,calibration_y);
        this.inSupplier = input;
        this.onFinished = onFinished;
        this.action = action;

        nt_x = new NTDouble(0.0, testtable, "calibration/"+name+"/x", (val)->{});
        nt_y = new NTDouble(0.0, testtable, "calibration/"+name+"/y", (val)->{});
        nt_ready = new NTBoolean(false, testtable, "calibration/"+name+"/ready", (val)->{});
        nt_action = new NTBoolean(false, testtable, "calibration/"+name+"/action", (val)->{});
        nt_done = new NTBoolean(false, testtable, "calibration/"+name+"/done", (val)->{});

        calibrationCmd = Commands.sequence(
            Commands.waitUntil(()->{
                var _r = ready.getAsBoolean();
                nt_ready.set(_r); 
                return _r;
            }),
            Commands.waitUntil(()->{
                double val=inSupplier.getAsDouble();
                nt_x.set(val);
                nt_y.set(get(val));
                // Wait until input is within calibration map X values
                if (val < m_xmin || val > m_xmax){
                    nt_action.set(false);
                    return false;
                }
                nt_action.set(true);
                // Wait until action is successful.
                return action.test(get(input.getAsDouble()));
            }),
            Commands.sequence(
                new InstantCommand(()->nt_done.set(true)),
                onFinished
            )
        );
        calibrationCmd.ignoringDisable(true).schedule();
    }

    public static Calibrate motor(String name, double [] calibration_x, double [] calibration_y, Motor motor, DoubleSupplier input){
        return motor(name, calibration_x, calibration_y, motor, input, Commands.sequence());
    }
    public static Calibrate motor(String name, double [] calibration_x, double [] calibration_y, Motor motor, DoubleSupplier input, Command onFinished){
        return new Calibrate(name,
            calibration_x,
            calibration_y,
            ()->motor.isSetupDone(),
            input,
            (val)->motor.setEncoderPosition(val),
            onFinished
        );
    }
}
