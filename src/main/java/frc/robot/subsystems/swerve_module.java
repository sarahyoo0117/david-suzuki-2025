package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.configs;
import frc.robot.constants;
import frc.robot.configs.swerve.module_config;

public class swerve_module {
    private final module_config config; 
    private final TalonFX drive;
    private final TalonFX turn; 
    private final DutyCycleEncoder abs;

    public swerve_module(module_config config) {
        this.config = config;
        drive = new TalonFX(config.can_drive.id, config.can_drive.canbus);
        turn = new TalonFX(config.can_turn.id, config.can_turn.canbus);
        drive.getConfigurator().apply(configs.swerve.drive_config(config.drive_inverted));
        turn.getConfigurator().apply(configs.swerve.turn_config(config.turn_inverted));
        abs = new DutyCycleEncoder(config.abs_channel);
        abs.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0); //use doubles to prevent integer rounding
        abs.setInverted(config.abs_inverted);
    }

    public void apply_state(SwerveModuleState state) {
        state.optimize(get_turn_position());
        drive.set(state.speedMetersPerSecond / 4.3);
        //no longer use pid velocity voltage because it's complicated to tune
        //TODO: fix units conversion issues ...
        //drive.setControl(new VelocityVoltage(Units.radiansToRotations(state.speedMetersPerSecond / constants.swerve.wheel_radius)));
        turn.setControl(new PositionVoltage(state.angle.getRotations()));
    } 

    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }

    public void zero_abs() {
        turn.setPosition(abs.get() - config.abs_offset);
    }

    public double get_distance_m() { 
        return constants.swerve.wheel_radius * drive.getPosition().getValue().in(Radians); 
    }

    public Rotation2d get_turn_position() {
        return Rotation2d.fromRadians(turn.getPosition().getValue().in(Radians));
    } 

    public AngularVelocity get_drive_velocity() {
        return drive.getVelocity().getValue();
    }

    public double get_drive_velocity_mps() {
        return get_drive_velocity().in(RadiansPerSecond) * constants.swerve.wheel_radius;
    }

    public double get_abs_raw() {
        return abs.get();
    }

    public SwerveModulePosition get_position() {
        return new SwerveModulePosition(get_distance_m(), get_turn_position());
    }

    public SwerveModuleState get_state() {
        return new SwerveModuleState(get_drive_velocity_mps(), get_turn_position()); 
    } 
}
