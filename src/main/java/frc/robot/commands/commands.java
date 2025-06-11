package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bindings;
import frc.robot.constants.elevator.elevator_state;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.end_effector;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.end_effector.gamepiece;
import frc.robot.subsystems.ramp;

public class commands {
    //TODO: address static friction
    public static final Command teleop_drive(swerve swerve, Supplier<Translation2d> drive_func, Supplier<Translation2d> turn_func) {
        final double drive_factor = 1.5;
        final double steer_factor = 3.5;
        return swerve.set_speeds(() -> {
            Translation2d left = drive_func.get();
            Translation2d right = turn_func.get();
            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                //x-axis is vertical, y-axis is horizontal
                left.getY() * drive_factor,
                left.getX() * drive_factor,
                right.getX() * steer_factor
            ), swerve.get_heading());
        }); 
    }

    //TODO: why elevator set state with deferred proxy??
    public static Command intake_algae(end_effector end_effector, elevator elevator) {
        return end_effector.cmd_intake(gamepiece.ALGAE)
            .alongWith(elevator.cmd_deferred_proxy_hold_state(bindings.elevator_height_to_intake_algae));
    }

    //TODO: test and fix intake coral command
    public static Command intake_coral(ramp ramp, end_effector end_effector, elevator elevator) {
        return Commands.parallel(
            ramp.cmd_intake(),
            end_effector.cmd_intake(gamepiece.CORAL),
            elevator.cmd_set_state(elevator_state.HOME)
        )
        .until(() -> !end_effector.lidar_sees_coral());
    }

    public static Command prescore(end_effector end_effector, elevator elevator) {
        if (end_effector.last_gamepiece == gamepiece.CORAL) {
            return elevator.cmd_deferred_proxy_hold_state(bindings.elevator_height_to_score_coral);
        } else {
            return elevator.cmd_deferred_proxy_hold_state(bindings.elevator_height_to_score_algae);
        }
    }
}
