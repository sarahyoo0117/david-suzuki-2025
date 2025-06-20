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

    //driver xbos controller buttons
    var ctrl_elevator_to_b = oi.cmd_driver.b(); //L1
    var ctrl_elevator_to_a = oi.cmd_driver.a(); //L2
    var ctrl_elevator_to_x = oi.cmd_driver.x(); //L3
    var ctrl_elevator_to_y = oi.cmd_driver.y(); //L4
    var ctrl_swerve_zero_abs = oi.cmd_driver.povUp().or(oi.cmd_xkeys.button(11));
    var ctrl_swerve_zero_heading = oi.cmd_driver.povDown();
    var ctrl_auto_align_left = oi.cmd_driver.back();
    var ctrl_auto_align_right = oi.cmd_driver.start();
    var ctrl_intake_algae = oi.cmd_driver.leftTrigger();
    var ctrl_intake_coral = oi.cmd_driver.rightTrigger();
    var ctrl_prescore = oi.cmd_driver.leftBumper();
    var ctrl_spit = oi.cmd_driver.rightBumper();
    //can climb only all elevator buttons are pressed lol
    var ctrl_climb_prep = ctrl_elevator_to_b.and(ctrl_elevator_to_a).and(ctrl_elevator_to_x).and(ctrl_elevator_to_y);
    //TODO: add ctrl_uppies and ctrl_tap when climb is prepped
    //var ctrl_elevator_move = oi.cmd_driver.rightBumper();

    //xkeys
    var ctrl_manual_up = oi.cmd_xkeys.button(12);    
    var ctrl_manual_down = oi.cmd_xkeys.button(13);    
    var ctrl_manual_pivot_up = oi.cmd_xkeys.button(14);    
    var ctrl_manual_pivot_down = oi.cmd_xkeys.button(15);    
    var ctrl_cease_manual = oi.cmd_xkeys.button(16);    
    var ctrl_zero_elevator = oi.cmd_xkeys.button(17);    
    var ctrl_zero_end_effector_pivot = oi.cmd_xkeys.button(18);    
    var ctrl_trap = oi.cmd_xkeys.button(19);    
  
    //binds commands to driver controller
    ctrl_elevator_to_b.onTrue(change_state(elevator_state.L1, elevator_state.ALGAE_REEF1, elevator_state.ALGAE_PROCESSOR));
    ctrl_elevator_to_a.onTrue(change_state(elevator_state.L2, elevator_state.ALGAE_REEF1, elevator_state.ALGAE_PROCESSOR));
    ctrl_elevator_to_x.onTrue(change_state(elevator_state.L3, elevator_state.ALGAE_REEF2, elevator_state.ALGAE_NET));
    ctrl_elevator_to_y.onTrue(change_state(elevator_state.L4, elevator_state.ALGAE_REEF2, elevator_state.ALGAE_NET));
    ctrl_swerve_zero_abs.onTrue(robot.swerve.cmd_zero_abs());
    ctrl_swerve_zero_heading.onTrue(robot.swerve.cmd_zero_heading());
    ctrl_auto_align_left.or(ctrl_auto_align_right).onTrue(robot.swerve.auto_align());
    ctrl_intake_algae.onTrue(commands.intake_algae(robot.end_effector, robot.elevator));
    ctrl_intake_coral.onTrue(commands.intake_coral(robot.ramp, robot.end_effector, robot.elevator));

    ctrl_auto_align_left.onTrue(robot.swerve.strafe_to_visible_tag(configs.ll_left, 4.0, 0.01));

    ctrl_prescore.onTrue(Commands.deferredProxy(() -> commands.prescore(robot.end_effector, robot.elevator)));
    ctrl_prescore.onFalse(Commands.deferredProxy(() -> robot.elevator.cmd_hold_state(elevator_state.HOME).alongWith(robot.end_effector.cmd_zero_pos_and_speed())));

    ctrl_spit.onTrue(robot.end_effector.cmd_spit().andThen(Commands.idle(robot.end_effector)));
    ctrl_spit.onFalse(robot.end_effector.cmd_zero_pos_and_speed());

    //binds commands to xkeys
    /* TODO: try this without ctrl_cease_manual
    var manual = ctrl_manual_up.or(ctrl_manual_down);
    manual.onTrue(robot.elevator.cmd_manual(oi.manual_input(ctrl_manual_up, ctrl_manual_down)));
     */
    var elevator_manual = robot.elevator.cmd_manual(oi.manual_input(ctrl_manual_up, ctrl_manual_down));
    ctrl_manual_up.or(ctrl_manual_down).onTrue(Commands.runOnce(() -> {
      elevator_manual.schedule();
    }, robot.elevator));

    var end_effector_manual = robot.end_effector.cmd_manual(oi.manual_input(ctrl_manual_pivot_up, ctrl_manual_pivot_down));
    ctrl_manual_pivot_up.or(ctrl_manual_pivot_down).onTrue(Commands.runOnce(() -> {
      end_effector_manual.schedule();
    }, robot.end_effector));

    ctrl_cease_manual.onTrue(Commands.runOnce(() -> {
      if (elevator_manual.isScheduled()) elevator_manual.cancel();
      if (end_effector_manual.isScheduled()) end_effector_manual.cancel();
    }));

    ctrl_zero_elevator.onTrue(robot.elevator.cmd_zero());
    ctrl_zero_end_effector_pivot.onTrue(robot.end_effector.cmd_zero());
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
