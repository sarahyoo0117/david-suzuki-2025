package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve;

public class commands {
    public static final Command teleop_drive(swerve swerve, Supplier<Translation2d> drive_func, Supplier<Double> turn_func) {
        final double drive_factor = 3;
        final double steer_factor = 3;
        return swerve.set_speeds(() -> {
            Translation2d left = drive_func.get();
            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                left.getX() * drive_factor, 
                left.getY() * drive_factor, 
                turn_func.get() * steer_factor
            ), swerve.get_heading());
        }); 
    }
}
