package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;

import static frc.robot.constants.end_effector.*;

public class end_effector extends SubsystemBase {
    private final TalonFX roller = new TalonFX(configs.can_end_effector_roller.id, configs.can_end_effector_roller.canbus); 
    private final TalonFX pivot = new TalonFX(configs.can_end_effector_pivot.id, configs.can_end_effector_pivot.canbus);
    private DigitalInput lidar = new DigitalInput(configs.end_effector_lidar);
    private boolean lidar_sees_coral = false; //TODO: unjam coral?
    private VelocityVoltage roller_output_req = new VelocityVoltage(0);
    private PositionVoltage pivot_output_req = new PositionVoltage(0); 
    private StatusSignal<Angle> pivot_position_signal = pivot.getPosition();

    public gamepiece last_gamepiece; 

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(pivot_position_signal);
        lidar_sees_coral = lidar.get();
    }
    
    @Override
    public void simulationPeriodic() {
        elevator.sim.update_wrist_angle(pivot_output_req.getPositionMeasure(), pivot_position_signal.getValue());
    }

    public boolean lidar_sees_coral() {
        return lidar_sees_coral;
    }

    public void set_roller_speed(AngularVelocity speed) {
        if (speed.abs(DegreesPerSecond) < 1) {
            roller.stopMotor();
        } else {
            roller.setControl(roller_output_req.withVelocity(speed));
        }
    }

    public void set_pivot_pos(Angle angle) {
       pivot.setControl(pivot_output_req.withPosition(angle));
    }

    public Command cmd_set_roller(AngularVelocity roller_speed) {
        return Commands.runOnce(() -> {
            set_roller_speed(roller_speed);
        }, this);
    }

    public Command cmd_set_pivot(Angle angle) {
        return Commands.runOnce(() -> {
            set_pivot_pos(angle);
        }, this);
    }

    public Command intake(gamepiece gp) {
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

    public Command spit(gamepiece gp) {
        return Commands.runOnce(() -> {
            switch (gp) {
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
