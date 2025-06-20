package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;
import frc.robot.constants.elevator.elevator_state;
import frc.robot.sim.elevator_mech2d;

//TODO: homming cmd 
//TODO: elevator sim 
//TODO: add elevator motors status signal 
public class elevator extends SubsystemBase {
    private final TalonFX motor_left = new TalonFX(configs.can_elevator_motor_left.id, configs.can_elevator_motor_left.canbus);
    private final TalonFX motor_right = new TalonFX(configs.can_elevator_motor_right.id, configs.can_elevator_motor_right.canbus);

    private MotionMagicVoltage position_request = new MotionMagicVoltage(0);
    private elevator_state target_state;
    private Distance manual_height = Meters.of(0);

    public static final elevator_mech2d sim = new elevator_mech2d(3, 3); 

    public elevator() {
        motor_left.getConfigurator().apply(configs.elevtor.elevator_left_config());
        motor_right.getConfigurator().apply(configs.elevtor.elevator_right_config());
        zero();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator_left_height", motor_left.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator_right_height", motor_right.getPosition().getValueAsDouble());
        ControlRequest elevator_req = position_request.withPosition(0);

        if (target_state != null) {
            if (target_state == elevator_state.MANUAL) {
                elevator_req = position_request.withPosition(manual_height.magnitude());
            } else {
                elevator_req = position_request.withPosition(target_state.height.magnitude());
            }
        }

        motor_left.setControl(elevator_req);
        motor_right.setControl(elevator_req);
    }
    
    @Override
    public void simulationPeriodic() {
        sim.update_elevator_pos((target_state == elevator_state.MANUAL) ? manual_height : target_state.height, get_height()); 
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

    public Distance get_height() {
        return Meters.of(motor_left.getPosition().getValueAsDouble());
    }

    public boolean height_at_target(elevator_state target) {
        return get_height() == target.height;
    }

    public Command cmd_manual(Supplier<Integer> height_speed) {
        return Commands.runOnce(() -> {
            target_state = elevator_state.MANUAL;
            int h = height_speed.get();
            manual_height = get_height().plus(Meters.of(0.01).times(h));
        }, this);
    }

    public Command cmd_zero() {
        return Commands.runOnce(() -> {
            zero();
        }, this);
    }
    
    public Command cmd_set_state(elevator_state state) {
        return Commands.runOnce(() -> {
            set_target_state(state);
        }, this);
    }

    public Command cmd_hold_state(elevator_state state) {
        return cmd_set_state(state).andThen(Commands.idle(this));
    }

    public Command cmd_deferred_proxy_set_state(elevator_state state) {
        return Commands.deferredProxy(() -> cmd_set_state(state));
    }
}
