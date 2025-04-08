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

    public static final Translation2d get_left_stick() {
        return new Translation2d(-xbox.getLeftY(), -xbox.getLeftX());
    }
}
