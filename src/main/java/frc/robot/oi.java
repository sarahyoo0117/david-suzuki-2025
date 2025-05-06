package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class oi {
    public static final CommandXboxController cmd_xbox = new CommandXboxController(0); 
    public static final XboxController xbox = cmd_xbox.getHID();
    public static final CommandGenericHID cmd_xkeys = new CommandGenericHID(1);
    public static final GenericHID xkeys = cmd_xkeys.getHID();

    public static Translation2d get_left_stick() {
        return new Translation2d(-xbox.getLeftY(), -xbox.getLeftX());
    }

    public static double simple_deadband(double input) {
        if (input <= 0.1) {
            return 0;
        }
        return input;
    }

    public static double linear_scaled_deadband(double input, double deadband) {
        double max = 1.0;
        double sign = Math.abs(input) / input;
        return (input - sign * deadband) / (max - deadband);
    }

    public static double cubic_scaled_deadband(double input, double weight, double deadband) {
        double max = 1.0;
        double input_cubic = weight * Math.pow(input,3) + (1 - weight) * input;
        double db_cubic = weight * Math.pow(deadband, 3) + (max-weight) * deadband;
        double sign = Math.abs(input) / input;
        return (input_cubic - sign * db_cubic) / (max - db_cubic);
    }
}
