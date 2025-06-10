package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;

import static frc.robot.constants.ramp.*;

public class ramp extends SubsystemBase {
    private final TalonFX roller = new TalonFX(configs.can_ramp_roller.id, configs.can_ramp_roller.canbus); 
    private final DigitalInput lidar_edge = new DigitalInput(configs.ramp_lidar_edge);
    private final DigitalInput lidar_middle = new DigitalInput(configs.ramp_lidar_middle);
    private boolean coral_homed = false, has_coral = false;
    private VelocityVoltage velocity_output_req = new VelocityVoltage(0);

    @Override
    public void periodic() {
        if (lidar_edge.get() || lidar_middle.get()) {
            has_coral = true;
        }
        if (lidar_edge.get() && lidar_middle.get()) {
            coral_homed = true;
        }
    }

    public boolean has_coral() {
        return has_coral;
    }

    public boolean coral_homed() {
        return coral_homed;
    }

    public void set_roller_speed(AngularVelocity speed) {
        roller.setControl(velocity_output_req.withVelocity(speed));
    }

    public Command cmd_set_roller(AngularVelocity speed) {
        return Commands.runOnce(() -> {
            set_roller_speed(speed);
        }, this);
    }

    public Command intake() {
        return cmd_set_roller(intake_coral);
    }

    //TODO: add unjam command
    public Command unjam() {
        has_coral = false;
        coral_homed = false;
        return cmd_set_roller(unjam_coral);
    }
}
