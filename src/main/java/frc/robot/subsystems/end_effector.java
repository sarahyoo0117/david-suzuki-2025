package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.bindings;
import frc.robot.configs;
import frc.robot.constants;
import frc.robot.constants.elevator.elevator_state;

import static frc.robot.constants.end_effector.*;

import java.util.function.Supplier;

//TODO: add end effector motors status signal if necessary
public class end_effector extends SubsystemBase {
    private final TalonFX roller = new TalonFX(configs.can_end_effector_roller.id, configs.can_end_effector_roller.canbus); 
    private final TalonFX pivot = new TalonFX(configs.can_end_effector_pivot.id, configs.can_end_effector_pivot.canbus);
    private final DigitalInput lidar = new DigitalInput(configs.end_effector_lidar);
    private final Debouncer lidar_debouncer = new Debouncer(0.03);
    private boolean lidar_sees_coral = false;

    private VelocityVoltage roller_velocity_request = new VelocityVoltage(0);
    private MotionMagicVoltage pivot_position_request = new MotionMagicVoltage(0); 
    private TorqueCurrentFOC hold_algae_request = new TorqueCurrentFOC(hold_algae_current);

    public gamepiece last_gamepiece = gamepiece.CORAL; 

    public end_effector() {
        pivot.setPosition(pivot_zero);
        roller.getConfigurator().apply(configs.end_effector.roller_config());
        
        setDefaultCommand(Commands.run(() -> {
            switch (last_gamepiece) {
                case CORAL:
                    set_feed(RotationsPerSecond.of(0));
                    break;
                case ALGAE:
                    roller.setControl(hold_algae_request); 
                    break;
            } 
        }, this));
    }

    @Override
    public void periodic() {
        lidar_sees_coral = lidar_debouncer.calculate(lidar_sees_coral_raw());
        SmartDashboard.putNumber("end_effector_pivot", pivot.getPosition().getValue().in(Degrees));
        SmartDashboard.putBoolean("lidar-sees-coral", lidar_sees_coral());
        SmartDashboard.putBoolean("cmd_triggered", cmd_triggered);
    }
    
    @Override
    public void simulationPeriodic() {
        elevator.sim.update_wrist_angle(pivot_position_request.getPositionMeasure(), pivot_position_request.getPositionMeasure());
    }

    public boolean lidar_sees_coral_raw() {
        return !lidar.get(); 
    }

    public boolean lidar_sees_coral() {
        return lidar_sees_coral;
    }

    public Angle get_pivot() {
        return pivot_position_request.getPositionMeasure();
    }

    public void set_feed(AngularVelocity speed) {
        if (speed.abs(DegreesPerSecond) < 1) {
            roller.stopMotor();
        } else {
            roller.setControl(roller_velocity_request.withVelocity(speed));
        }
    }

    public void set_pivot(Angle angle) {
       pivot.setControl(pivot_position_request.withPosition(angle));
    }

    public void zero() {
        roller.setControl(roller_velocity_request.withVelocity(0));
        pivot.setControl(pivot_position_request.withPosition(pivot_zero));
    }

    public Command cmd_manual(Supplier<Integer> pivot_speed) {
        return Commands.runOnce(() -> {
            int p = pivot_speed.get();
            Angle manual_pivot = get_pivot().plus(Degrees.of(p));
            set_pivot(manual_pivot);
        }, this);
    }

    public Command cmd_set_feed(AngularVelocity roller_speed) {
        return Commands.runOnce(() -> {
            set_feed(roller_speed);
        }, this);
    }

    public Command cmd_set_pivot(Angle angle) {
        return Commands.runOnce(() -> {
            set_pivot(angle);
        }, this);
    }

    public Command cmd_zero() {
        return Commands.runOnce(() -> {
            zero();
        }, this);
    }

    public Command cmd_zero_pos_and_speed() {
        return Commands.runOnce(() -> {
            set_pivot(pivot_zero);
            set_feed(RotationsPerSecond.of(0));
        }, this);
    }

    public Command cmd_intake(gamepiece gp) {
        return Commands.run(() -> {
            last_gamepiece = gp;
            switch (gp) {
                case CORAL:
                    set_feed(intake_coral); 
                    break;
                case ALGAE:
                    if (bindings.elevator_height_to_intake_algae == elevator_state.ALGAE_REEF1 || bindings.elevator_height_to_intake_algae == elevator_state.ALGAE_REEF2) {
                        set_pivot(pivot_intake_algae_reef);
                    } else {
                        set_pivot(pivot_intake_algae_ground);
                    }
                    set_feed(intake_algae);
                    break;
            }
        }, this);
    }

    boolean cmd_triggered = false;
    public Command cmd_intake_with_lidar() {
        return Commands.run(() -> {
            set_feed(constants.end_effector.intake_precise);
            last_gamepiece = gamepiece.CORAL;
            cmd_triggered = true;
        }, this)
        .until(() -> !lidar_sees_coral_raw());
    }

    public Command cmd_spit() {
        return Commands.runOnce(() -> {
            switch (last_gamepiece) {
                case CORAL:
                    if (bindings.elevator_height_to_score_coral == elevator_state.L1) {
                        set_pivot(pivot_score_coral_L1);
                    }
                    set_feed(spit_coral);    
                    break;
                case ALGAE:
                    set_feed(spit_algae); 
                    break;
            }
        }, this);
    }

    public enum gamepiece {
        CORAL,
        ALGAE
    }
}
