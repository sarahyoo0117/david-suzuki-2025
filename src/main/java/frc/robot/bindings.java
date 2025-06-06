package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.elevator.elevator_state;
import frc.robot.commands.commands;

import frc.robot.oi.shaping_chooser;

public final class bindings {
  //stragety: score coral -> get algae from reef -> score to net / processor
  public static elevator_state elevator_height_to_score_coral = elevator_state.HOME;
  public static elevator_state elevator_height_to_intake_algae = elevator_state.HOME;
  public static elevator_state elevator_height_to_score_algae = elevator_state.HOME;

  private static final shaping_chooser drive_shaping_chooser = new shaping_chooser("drive_shaping_chooser");
  private static final shaping_chooser turn_shaping_chooser = new shaping_chooser("turn_shaping_chooser");
  private static final Supplier<Translation2d> drive_func = () -> oi.vector_deadband(oi.get_left_stick(), drive_shaping_chooser::shape);
  private static final Supplier<Translation2d> turn_func = () -> oi.vector_deadband(oi.get_right_stick(), turn_shaping_chooser::shape);

  private final robot robot;

  public bindings(robot robot) {
    this.robot = robot;
    configureBindings();
  }

  private void configureBindings() {

    //subsystem default commands
    robot.swerve.setDefaultCommand(commands.teleop_drive(robot.swerve, drive_func, turn_func));
    robot.elevator.setDefaultCommand(robot.elevator.cmd_set_state(elevator_state.HOME));
    robot.end_effector.setDefaultCommand(robot.end_effector.zero());
    
    //controller
    var ctrl_intake_algae_ground = oi.cmd_driver.rightTrigger();
    //var ctrl_intake_algae_reef = oi.cmd_driver.leftTrigger();
    var ctrl_score_coral_L1 = oi.cmd_driver.leftTrigger();
    var ctrl_elevator_to_b = oi.cmd_driver.b(); //L1
    var ctrl_elevator_to_a = oi.cmd_driver.a(); //L2
    var ctrl_elevator_to_x = oi.cmd_driver.x(); //L3
    var ctrl_elevator_to_y = oi.cmd_driver.y(); //L4
    var ctrl_elevator_move = oi.cmd_driver.rightBumper();
    
    //xkeys
    var xkey_swerve_zero = oi.cmd_xkeys.button(11);
    xkey_swerve_zero.onTrue(robot.swerve.zero());

    ctrl_intake_algae_ground.whileTrue(commands.intake_algae_ground(robot.end_effector));
    ctrl_score_coral_L1.whileTrue(commands.score_coral_L1(robot.end_effector, robot.elevator));

    ctrl_elevator_to_b.onTrue(change_state(elevator_state.L1, elevator_state.ALGAE_REEF1, elevator_state.ALGAE_PROCESSOR));
    ctrl_elevator_to_a.onTrue(change_state(elevator_state.L2, elevator_state.ALGAE_REEF1, elevator_state.ALGAE_PROCESSOR));
    ctrl_elevator_to_x.onTrue(change_state(elevator_state.L3, elevator_state.ALGAE_REEF2, elevator_state.ALGAE_NET));
    ctrl_elevator_to_y.onTrue(change_state(elevator_state.L4, elevator_state.ALGAE_REEF2, elevator_state.ALGAE_NET));
    ctrl_elevator_move.whileTrue(
      Commands.deferredProxy(() -> robot.elevator.hold_target_state(elevator_height_to_score_coral))
    );
  }

  //TODO: LEDs
  public final Command change_state(elevator_state coral, elevator_state algae_intake, elevator_state algae_score) {
      return Commands.runOnce(() -> {
          elevator_height_to_score_coral = coral;
          elevator_height_to_intake_algae = algae_intake;
          elevator_height_to_score_algae = algae_score;
      }, robot.elevator);
  }
}
