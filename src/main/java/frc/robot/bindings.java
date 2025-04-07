package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.commands;

public final class bindings {
  public void configureBindings(robot robot) {
    Supplier<Translation2d> drive_func = () -> oi.get_left_stick();
    Supplier<Double> turn_func = () -> -oi.drive.getRightX();
    robot.swerve.setDefaultCommand(commands.teleop_drive(robot.swerve, drive_func, turn_func));
  }
}
