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

  private final robot robot;

  public bindings(robot robot) {
    this.robot = robot;
    configureBindings();
  }

  private void configureBindings() {
    //swerve 
    final shaping_chooser drive_shaping_chooser = new shaping_chooser("drive_shaping_chooser");
    final shaping_chooser turn_shaping_chooser = new shaping_chooser("turn_shaping_chooser");
    final Supplier<Translation2d> drive_func = () -> oi.vector_deadband(oi.get_left_stick(), drive_shaping_chooser::shape);
    final Supplier<Translation2d> turn_func = () -> oi.vector_deadband(oi.get_right_stick(), turn_shaping_chooser::shape);

    //subsystem default commands
    robot.swerve.setDefaultCommand(commands.teleop_drive(robot.swerve, drive_func, turn_func));
    robot.elevator.setDefaultCommand(robot.elevator.cmd_set_state(elevator_state.HOME));
    robot.end_effector.setDefaultCommand(robot.end_effector.zero());

    //driver xbos controller buttons
    var ctrl_swerve_zero_abs = oi.cmd_driver.povUp().or(oi.cmd_xkeys.button(11));
    var ctrl_swerve_zero_heading = oi.cmd_driver.povDown();
    var ctrl_auto_align_left = oi.cmd_driver.back();
    var ctrl_auto_align_right = oi.cmd_driver.start();
    var ctrl_intake_coral = oi.cmd_driver.rightTrigger();
    var ctrl_prescore = oi.cmd_driver.leftBumper();
    var ctrl_spit = oi.cmd_driver.rightBumper();
    var ctrl_intake_algae = oi.cmd_driver.leftTrigger();
    var ctrl_elevator_to_b = oi.cmd_driver.b(); //L1
    var ctrl_elevator_to_a = oi.cmd_driver.a(); //L2
    var ctrl_elevator_to_x = oi.cmd_driver.x(); //L3
    var ctrl_elevator_to_y = oi.cmd_driver.y(); //L4
    //can climb only all elevator buttons are pressed lol
    var ctrl_climb_prep = ctrl_elevator_to_b.and(ctrl_elevator_to_a).and(ctrl_elevator_to_x).and(ctrl_elevator_to_y);
    //TODO: add ctrl_uppies and ctrl_tap when climb is prepped
    //var ctrl_elevator_move = oi.cmd_driver.rightBumper();

    //xkeys
    var ctrl_manual_elevator_up = oi.cmd_xkeys.button(12);    
    var ctrl_manual_elevator_down = oi.cmd_xkeys.button(13);    
    var ctrl_manual_end_effector_pivot_up = oi.cmd_xkeys.button(14);    
    var ctrl_manual_end_effector_pivot_down = oi.cmd_xkeys.button(15);    
    var ctrl_cease_manual = oi.cmd_xkeys.button(16);    
    var ctrl_zero_elevator = oi.cmd_xkeys.button(17);    
    var ctrl_zero_end_effector_pivot = oi.cmd_xkeys.button(18);    
    var ctrl_trap = oi.cmd_xkeys.button(19);    
  
    ctrl_swerve_zero_abs.onTrue(robot.swerve.cmd_zero_abs());
    ctrl_swerve_zero_heading.onTrue(robot.swerve.cmd_zero_heading());

    ctrl_elevator_to_b.onTrue(change_state(elevator_state.L1, elevator_state.ALGAE_REEF1, elevator_state.ALGAE_PROCESSOR));
    ctrl_elevator_to_a.onTrue(change_state(elevator_state.L2, elevator_state.ALGAE_REEF1, elevator_state.ALGAE_PROCESSOR));
    ctrl_elevator_to_x.onTrue(change_state(elevator_state.L3, elevator_state.ALGAE_REEF2, elevator_state.ALGAE_NET));
    ctrl_elevator_to_y.onTrue(change_state(elevator_state.L4, elevator_state.ALGAE_REEF2, elevator_state.ALGAE_NET));
    /*
    ctrl_elevator_move.whileTrue(
      Commands.deferredProxy(() -> robot.elevator.hold_target_state(elevator_height_to_score_coral))
    );
     */
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
