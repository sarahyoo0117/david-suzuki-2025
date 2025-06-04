package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bindings;
import frc.robot.constants;
import frc.robot.constants.elevator.elevator_state;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.end_effector;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.end_effector.gamepiece;

public class commands {
    public static final Command teleop_drive(swerve swerve, Supplier<Translation2d> drive_func, Supplier<Double> turn_func) {
        final double drive_factor = 3;
        final double steer_factor = 6;
        return swerve.set_speeds(() -> {
            Translation2d left = drive_func.get();
            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                MathUtil.applyDeadband(left.getX() * drive_factor, 0.1),
                MathUtil.applyDeadband(left.getY() * drive_factor, 0.1),
                turn_func.get() * steer_factor
            ), swerve.get_heading());
        }); 
    }

    public static final Command intake_algae_ground(end_effector end_effector) {
        return Commands.runOnce(() -> {
            end_effector.set_pivot_pos(Degrees.of(0));
            end_effector.set_roller_speed(constants.end_effector.intake_algae);
        }, end_effector);
    }

    public static final Command intake_algae_reef(end_effector end_effector, elevator elevator) {
        return end_effector.intake(gamepiece.ALGAE)
        .alongWith(elevator.hold_target_state(bindings.elevator_height_to_intake_algae));
    }

    public static final Command score_coral_L1(end_effector end_effector, elevator elevator) {
        return Commands.parallel(
            elevator.hold_target_state(elevator_state.L1),
            end_effector.cmd_set_pivot(Degrees.of(-28)),
            end_effector.cmd_set_roller(constants.end_effector.intake_manual)
        );
    }
}
