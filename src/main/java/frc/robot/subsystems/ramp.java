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
    private VelocityVoltage velocity_output_req = new VelocityVoltage(0);

    public boolean coral_seated() {
        return lidar_edge.get() && lidar_middle.get(); 
    }

    public boolean has_coral() {
        return lidar_edge.get() || lidar_middle.get();
    }

    public void set_roller_speed(AngularVelocity speed) {
        roller.setControl(velocity_output_req.withVelocity(speed));
    }

    public Command cmd_set_roller_speed(AngularVelocity speed) {
        return Commands.runOnce(() -> {
            set_roller_speed(speed);
        }, this);
    }

    public Command cmd_intake() {
        return cmd_set_roller_speed(intake_coral);
    }

    //TODO: add unjam command
    public Command cmd_unjam() {
        return cmd_set_roller_speed(unjam_coral);
    }
}
