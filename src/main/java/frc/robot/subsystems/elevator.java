package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;
import frc.robot.constants;
import frc.robot.constants.elevator.elevator_state;

//TODO: homming cmd 
//TODO: elevator sim 
public class elevator extends SubsystemBase {
    private final TalonFX motor_left = new TalonFX(constants.ids.elevator_motor_left, configs.canbus);
    private final TalonFX motor_right = new TalonFX(constants.ids.elevator_motor_right, configs.canbus);
    private final DigitalInput homming_sensor = new DigitalInput(constants.ids.elevator_homming_sensor);
    private elevator_state target_state;
    MotionMagicVoltage position_request = new MotionMagicVoltage(0);

    @Override
    public void periodic() {
        ControlRequest elevator_req = position_request.withPosition(0);

        if (target_state != null) {
            elevator_req = position_request.withPosition(target_state.height);
        }

        motor_left.setControl(elevator_req);
        motor_right.setControl(elevator_req);
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

    public Command cmd_set_state(elevator_state state) {
        return Commands.runOnce(() -> {
            set_target_state(state);
        }, this);
    }

    public Command hold_target_state(elevator_state state) {
        return cmd_set_state(state).andThen(Commands.idle(this));
    }
}
