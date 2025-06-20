package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;

import static frc.robot.constants.ramp.*;

public class ramp extends SubsystemBase {
    private final TalonFX roller = new TalonFX(configs.can_ramp_roller.id, configs.can_ramp_roller.canbus); 
    private final DutyCycle lidar_edge = new DutyCycle(new DigitalInput(configs.ramp_lidar_edge));
    private final DutyCycle lidar_middle = new DutyCycle(new DigitalInput(configs.ramp_lidar_middle));
    private final Debouncer lidar_edge_debouncer = new Debouncer(0.04, DebounceType.kBoth);
    private final Debouncer lidar_middle_debouncer = new Debouncer(0.02, DebounceType.kBoth);
    
    private boolean filtered_lidar_middle = false, filtered_lidar_edge = false;
    private VelocityVoltage velocity_output_req = new VelocityVoltage(0);

    public ramp() {
       setDefaultCommand(hold());
    }
    
    @Override
    public void periodic() {
        filtered_lidar_edge = lidar_edge_debouncer.calculate(get_lidar_edge_raw());
        filtered_lidar_middle = lidar_middle_debouncer.calculate(get_lidar_middle_raw());
        SmartDashboard.putBoolean("ramp_lidar_edge", filtered_lidar_edge);
        SmartDashboard.putBoolean("ramp_lidar_middle", filtered_lidar_middle);
    }

    public boolean get_lidar_middle_raw() {
        var high_time = lidar_middle.getHighTimeNanoseconds();
        return 1000000 < high_time && high_time < 1350000;
    }

    public boolean get_lidar_edge_raw() {
        var high_time = lidar_edge.getHighTimeNanoseconds();
        return 1000000 < high_time && high_time < 1400000;
    }

    public void set_roller_speed(AngularVelocity speed) {
        roller.setControl(velocity_output_req.withVelocity(speed));
    }

    public void stop() {
        roller.stopMotor();
    }

    public Command hold() {
        return Commands.run(() -> {
            if (filtered_lidar_edge && filtered_lidar_middle) {
                stop();
            } else if(filtered_lidar_edge && !filtered_lidar_middle) {

            } else if (!filtered_lidar_edge && filtered_lidar_middle) {

            } else {

            }
        }, this);
    }

    public Command cmd_set_roller_speed(AngularVelocity speed) {
        return Commands.runOnce(() -> {
            set_roller_speed(speed);
        }, this);
    }

    public Command cmd_intake() {
        return cmd_set_roller_speed(intake);
    }

    //TODO: add unjam command
    public Command cmd_unjam() {
        return cmd_set_roller_speed(intake);
    }
}
