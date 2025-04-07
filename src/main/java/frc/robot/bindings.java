package frc.robot;

import frc.robot.commands.commands;

public final class bindings {
  public void configureBindings(robot robot) {
    robot.swerve.setDefaultCommand(commands.teleop_drive(robot.swerve, oi.get_left_stick(), -oi.drive.getRightX()));
  }
}
