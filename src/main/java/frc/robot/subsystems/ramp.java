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
    private final TalonFX intake = new TalonFX(configs.ids.ramp_intake); 
    //TODO: use Duty Cycle for lidars
    private final DigitalInput lidar_edge = new DigitalInput(configs.ids.ramp_lidar_edge);
    private final DigitalInput lidar_middle =new DigitalInput(configs.ids.ramp_lidar_middle);
    private boolean coral_homed = false, has_coral = false;
    private VelocityVoltage output_req = new VelocityVoltage(0);
    //private AngularVelocity desired_vel = DegreesPerSecond.of(0); honestly why do we need desired 

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
            intake.setControl(output_req.withVelocity(speed));
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
