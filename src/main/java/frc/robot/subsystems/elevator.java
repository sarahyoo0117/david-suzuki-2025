package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;
import frc.robot.constants.elevator.elevator_state;
import frc.robot.sim.elevator_mech2d;

//TODO: homming cmd 
//TODO: elevator sim 
public class elevator extends SubsystemBase {
    private final TalonFX motor_left = new TalonFX(configs.can_elevator_motor_left.id, configs.can_elevator_motor_left.canbus);
    private final TalonFX motor_right = new TalonFX(configs.can_elevator_motor_right.id, configs.can_elevator_motor_right.canbus);
    MotionMagicVoltage position_request = new MotionMagicVoltage(0);
    private StatusSignal<AngularVelocity> velocity_signal = motor_left.getVelocity(); 
    private StatusSignal<Angle> position_signal = motor_left.getPosition();
    private elevator_state target_state;

    public static final elevator_mech2d sim = new elevator_mech2d(3, 3); //TODO: elevator sim

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position_signal, velocity_signal);
        ControlRequest elevator_req = position_request.withPosition(0);

        if (target_state != null) {
            elevator_req = position_request.withPosition(target_state.height.magnitude());
        }

        //TODO: I don't think it is good idea to request controls periodically. just make set methods like other subsystems?
        motor_left.setControl(elevator_req);
        motor_right.setControl(elevator_req);
        sim.update_elevator_pos(target_state.height, Meters.of(position_signal.getValueAsDouble())); //TODO: how to know actual elevator height
    }
    
    public void set_target_state(elevator_state state) {
        target_state = state;
    }

    public void stop_motors() {
        motor_left.stopMotor();
        motor_right.stopMotor();
    }

    public void zero() {
        motor_left.setPosition(0);
        motor_right.setPosition(0);
    }

    public Command home() {
        return cmd_set_state(elevator_state.HOME).andThen(Commands.idle(this));
    }

    public Command cmd_set_state(elevator_state state) {
        return Commands.runOnce(() -> {
            set_target_state(state);
        }, this);
    }

    public Command hold_target_state(elevator_state state) {
        return cmd_set_state(state).andThen(Commands.idle(this));
    }
}
