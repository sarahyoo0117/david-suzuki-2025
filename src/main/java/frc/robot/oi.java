package frc.robot;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class oi {
    public static final CommandXboxController cmd_driver = new CommandXboxController(0); 
    public static final XboxController driver = cmd_driver.getHID();
    public static final CommandGenericHID cmd_xkeys = new CommandGenericHID(1);
    public static final GenericHID xkeys = cmd_xkeys.getHID();

    //up-left is positive
    public static Translation2d get_left_stick() {
        return new Translation2d(-driver.getLeftY(), -driver.getLeftX());
    }

    public static Translation2d get_right_stick() {
        return new Translation2d(-driver.getRightY(), -driver.getRightX());
    }

    public static Translation2d vector_deadband(Translation2d input, double deadband, double max_length, DoubleUnaryOperator shaping_func) {
        double length = input.getNorm();
        if (length <= deadband) {
            return new Translation2d();
        } 
        Rotation2d angle = input.getAngle();
        double new_length = shaping_func.applyAsDouble((length - deadband) / (max_length - deadband));
        return new Translation2d(Math.min(new_length, max_length), angle);
    }
 
    public static Translation2d vector_deadband(Translation2d input, DoubleUnaryOperator shaping_func) {
        return vector_deadband(input, 0.1, 1, shaping_func);
    }

    public static Supplier<Integer> manual_input(Trigger up, Trigger down) {
        return () -> {
            int n = 0;
            if (up.getAsBoolean()) {
                n ++;
            }
            if (down.getAsBoolean()) {
                n --;
            }
            return n;
        };
    }

    public static class shaping_chooser {
        private final SendableChooser<String> chooser;
        private final String linear = "linear", squared = "squared", cubic = "cubic", custom = "custom";
        private final String[] options = {linear, squared, cubic, custom};

        public shaping_chooser(String pref_key) {
            chooser = new SendableChooser<String>();
            for (String option : options) {
                chooser.addOption(option, option);
            }
            String pref = Preferences.getString(pref_key, squared);
            chooser.setDefaultOption(pref, pref);
            chooser.onChange((String option) -> {
                Preferences.setString(pref_key, option);
            });
            SmartDashboard.putData(chooser);
        }

        //TODO: make custom shaping function
        public static double custom(double x) {
            return 1/5 * x * Math.pow(2, 2 * x);
        }

        public double shape(double x) {
            switch (chooser.getSelected()) {
                case squared:
                    return Math.pow(x, 2);
                case cubic:
                    return Math.pow(x, 3);
                case custom:
                    return custom(x);
                case linear:
                    return x;
                default:
                    return x; 
            }
        }
    }

    //TODO: test custom deadbands
    public static double linear_scaled_deadband(double input, double deadband) {
        double max = 1.0;
        double sign = Math.abs(input) / input;
        return (input - sign * deadband) / (max - deadband);
    }

    public static double cubic_scaled_deadband(double input, double weight, double deadband) {
        double input_cubic = weight * Math.pow(input,3) + (1 - weight) * input;
        double db_cubic = weight * Math.pow(deadband, 3) + (1 - weight) * deadband;
        double sign = Math.abs(input) / input;
        return (input_cubic - sign * db_cubic) / (1 - db_cubic);
    }
}
