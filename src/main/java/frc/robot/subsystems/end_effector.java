package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

public class end_effector extends SubsystemBase {
    private final TalonFX roller = new TalonFX(configs.can_end_effector_roller.id, configs.can_end_effector_roller.canbus); 
    private final TalonFX pivot = new TalonFX(configs.can_end_effector_pivot.id, configs.can_end_effector_pivot.canbus);
    private DigitalInput lidar = new DigitalInput(configs.end_effector_lidar);
    private boolean lidar_sees_coral = false;
    private VelocityVoltage roller_output_req = new VelocityVoltage(0);
    private PositionVoltage pivot_output_req = new PositionVoltage(0); 
    //TODO: what's the use of target variables
    private AngularVelocity target_roller_velocity = RotationsPerSecond.of(0); 
    private AngularVelocity target_pivot_velocity = DegreesPerSecond.of(0); 
    private StatusSignal<Angle> pivot_position_signal = pivot.getPosition();

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(pivot_position_signal);
        lidar_sees_coral = lidar.get();
        elevator.sim.update_wrist_angle(pivot_output_req.getPositionMeasure(), pivot_position_signal.getValue());
    }

    public boolean lidar_sees_coral() {
        return lidar_sees_coral;
    }

    public void set_roller_speed(AngularVelocity speed) {
        target_roller_velocity = speed;
        roller.setControl(roller_output_req.withVelocity(speed));
    }

    public void set_pivot_speed(AngularVelocity speed) {
       target_pivot_velocity = speed; 
       pivot.setControl(pivot_output_req.withVelocity(speed));
    }

    public Command set_roller(AngularVelocity roller_speed) {
        return Commands.runOnce(() -> {
            set_roller_speed(roller_speed);
        }, this);
    }

    public Command set_pivot(AngularVelocity pivot_speed) {
        return Commands.runOnce(() -> {
            set_pivot_speed(pivot_speed);
        }, this);
    }

    //TODO: different pivot pos depending on game pieces
    public Command feed() {
        return Commands.sequence(
            set_roller(RotationsPerSecond.of(1)),
            set_pivot(DegreesPerSecond.of(45))
        );
    }

    public Command unjam() {
        lidar_sees_coral = false;
        return set_roller(RotationsPerSecond.of(-1)); 
    }

    public Command spit() {
        return Commands.runOnce(() -> {
            lidar_sees_coral = false;
            pivot.setPosition(0);
        }, this)
        .andThen(
            set_roller(RotationsPerSecond.of(1))
        );
    }
}
