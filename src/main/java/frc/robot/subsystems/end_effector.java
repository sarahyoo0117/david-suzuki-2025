package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;

import static frc.robot.constants.end_effector.*;

import java.util.function.Supplier;

//TODO: add end effector motors status signal if necessary
public class end_effector extends SubsystemBase {
    private final TalonFX roller = new TalonFX(configs.can_end_effector_roller.id, configs.can_end_effector_roller.canbus); 
    private final TalonFX pivot = new TalonFX(configs.can_end_effector_pivot.id, configs.can_end_effector_pivot.canbus);
    private DigitalInput lidar = new DigitalInput(configs.end_effector_lidar);
    private VelocityVoltage roller_velocity_request = new VelocityVoltage(0);
    private MotionMagicVoltage pivot_position_request = new MotionMagicVoltage(0); 
    private Angle manual_pivot = Degrees.of(0);

    public gamepiece last_gamepiece; 

    public end_effector() {
        pivot.setPosition(pivot_zero);
        roller.getConfigurator().apply(configs.end_effector.roller_config());
        setDefaultCommand(Commands.run(() -> {
            switch (last_gamepiece) {
                case CORAL:
                    set_pivot(score_coral_L1_pivot);    
                    break;
                case ALGAE:
                    set_pivot(pivot_zero);
                    break;
            }
        }, this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("end_effector_pivot", pivot.getPosition().getValue().in(Degrees));
    }
    
    @Override
    public void simulationPeriodic() {
        elevator.sim.update_wrist_angle(pivot_position_request.getPositionMeasure(), pivot_position_request.getPositionMeasure());
    }

    public boolean lidar_sees_coral() {
        return lidar.get();
    }

    public Angle get_pivot() {
        return pivot_position_request.getPositionMeasure();
    }

    public void set_roller_speed(AngularVelocity speed) {
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
        set_roller_speed(RotationsPerSecond.of(0));
        set_pivot(pivot_zero);
    }

    public Command cmd_manual(Supplier<Integer> pivot_speed) {
        return Commands.runOnce(() -> {
            int p = pivot_speed.get();
            manual_pivot = get_pivot().plus(Degrees.of(p));
            set_pivot(manual_pivot);
        }, this);
    }

    public Command cmd_set_roller_speed(AngularVelocity roller_speed) {
        return Commands.runOnce(() -> {
            set_roller_speed(roller_speed);
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

    public Command cmd_intake(gamepiece gp) {
        return Commands.run(() -> {
            last_gamepiece = gp;
            switch (gp) {
                case CORAL:
                    set_roller_speed(intake_manual); 
                    break;
                case ALGAE:
                    set_roller_speed(intake_algae);
                    break;
            }
        }, this);
    }

    public Command cmd_spit() {
        return Commands.runOnce(() -> {
            switch (last_gamepiece) {
                case CORAL:
                    set_roller_speed(spit_coral);    
                    break;
                case ALGAE:
                    set_roller_speed(spit_algae); 
                    break;
            }
        }).andThen(Commands.idle(this));
    }

    public enum gamepiece {
        CORAL,
        ALGAE
    }
}
