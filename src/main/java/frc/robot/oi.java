package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class oi {
    public static final CommandXboxController cmd_ctrl = new CommandXboxController(0); 
    public static final XboxController drive = cmd_ctrl.getHID();
    public static final CommandGenericHID xkeys = new CommandGenericHID(1);

    public static final Translation2d get_left_stick() {
        return new Translation2d(-drive.getLeftY(), -drive.getLeftX());
    }
}
