package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve;

public class commands {
    public static final Command teleop_drive(swerve swerve, Translation2d inputs, double omega_input) {
        final double drive_factor = 3;
        final double steer_factor = 3;
        return swerve.set_speeds(() -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                inputs.getX() * drive_factor, 
                inputs.getY() * drive_factor, 
                omega_input * steer_factor
            ), swerve.get_heading());
        }); 
    }
}
