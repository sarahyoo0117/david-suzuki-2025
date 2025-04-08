package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.elevator.elevator_state;
import frc.robot.commands.commands;

public final class bindings {
  //stragety: score coral -> get algae from reef -> score to net / processor
  public static elevator_state height_for_coral = elevator_state.L4;
  public static elevator_state height_for_algae_intake = elevator_state.ALGAE_INTAKE2;
  public static elevator_state height_for_algae_score = elevator_state.ALGAE_NET;

  private final robot robot;
  public bindings(robot robot) {
    this.robot = robot;
    configureBindings();
  }

  private void configureBindings() {
    Supplier<Translation2d> drive_func = () -> oi.get_left_stick();
    Supplier<Double> turn_func = () -> -oi.drive.getRightX();
    robot.swerve.setDefaultCommand(commands.teleop_drive(robot.swerve, drive_func, turn_func));
    
    var ctrl_elevator_to_b = oi.cmd_ctrl.b(); //L1
    var ctrl_elevator_to_a = oi.cmd_ctrl.a(); //L2
    var ctrl_elevator_to_x = oi.cmd_ctrl.x(); //L3
    var ctrl_elevator_to_y = oi.cmd_ctrl.y(); //L4
  }

  public final Command change_state(elevator_state coral, elevator_state algae_intake, elevator_state algae_score) {
      return Commands.runOnce(() -> {
          height_for_coral = coral;
          height_for_algae_score = algae_intake;
          height_for_algae_score = algae_score;
      }, robot.elevator);
  }
}
