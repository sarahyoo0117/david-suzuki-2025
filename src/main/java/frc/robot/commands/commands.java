package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bindings;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.end_effector;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.end_effector.gamepiece;

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

    public static final Command intake_algae(end_effector end_effector, elevator elevator) {
        return end_effector.intake(gamepiece.ALGAE)
            .alongWith(Commands.run(() -> {
                elevator.set_target_state(bindings.elevator_height_to_score_algae);
            }, elevator));
    }
}
