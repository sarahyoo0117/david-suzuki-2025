package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;

public class ramp extends SubsystemBase {
    private final TalonFX roller = new TalonFX(configs.can_ramp_roller.id, configs.can_ramp_roller.canbus); 
    private final DigitalInput lidar_edge = new DigitalInput(configs.ramp_lidar_edge);
    private final DigitalInput lidar_middle = new DigitalInput(configs.ramp_lidar_middle);
    private boolean coral_homed = false, has_coral = false;
    private VelocityVoltage output_req = new VelocityVoltage(0);
    private AngularVelocity desired_vel = DegreesPerSecond.of(0);  

    @Override
    public void periodic() {
        if (lidar_edge.get() || lidar_middle.get()) {
            has_coral = true;
        }
        if (lidar_edge.get() && lidar_middle.get()) {
            coral_homed = true;
        }
    }

    public Command set(AngularVelocity speed) {
        return Commands.runOnce(() -> {
           // desired_vel = speed;
            roller.setControl(output_req.withVelocity(speed));
        }, this);
    }

    public Command feed() {
        return set(DegreesPerSecond.of(1));
    }

    public Command unjam() {
        has_coral = false;
        coral_homed = false;
        return set(DegreesPerSecond.of(-1));
    }

    public boolean has_coral() {
        return has_coral;
    }

    public boolean coral_homed() {
        return coral_homed;
    }
}
