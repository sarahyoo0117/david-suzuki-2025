package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.configs;
import frc.robot.constants;
import frc.robot.configs.swerve.module_config;

public class swerve_module {
    private final module_config module_config; 
    private TalonFX drive;
    private TalonFX turn; 
    private DutyCycleEncoder abs;

    public swerve_module(module_config module_config) {
        this.module_config = module_config;
        drive = new TalonFX(module_config.drive_id, configs.canbus);
        turn = new TalonFX(module_config.turn_id, configs.canbus);
        drive.getConfigurator().apply(configs.swerve.drive_config(module_config.drive_inverted));
        turn.getConfigurator().apply(configs.swerve.turn_config(module_config.turn_inverted));
        abs = new DutyCycleEncoder(module_config.abs_channel);
        abs.setDutyCycleRange(1 / 4096, 4096 / 4096);
        abs.setInverted(module_config.abs_inverted);
        reset_turn();
    }

    public void apply_state(SwerveModuleState state) {
        state.optimize(get_rotation2d());
        //TODO: drive and turn voltage
        drive.setControl(new VelocityVoltage(Units.radiansToRotations(state.speedMetersPerSecond / constants.swerve.wheel_radius)));
        turn.setControl(new PositionVoltage(state.angle.getRotations()));
    } 

    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }

    public void reset_turn() {
        turn.setPosition(abs.get() - module_config.abs_offset);
    }

    public Distance get_distance() { 
        return Meters.of(constants.swerve.wheel_radius).times(drive.getPosition().getValue().in(Radians)); 
    }

    public Rotation2d get_rotation2d() {
        return Rotation2d.fromRadians(turn.getPosition().getValue().in(Radians));
    } 

    public AngularVelocity get_drive_velocity() {
        return drive.getVelocity().getValue();
    }

    public double get_drive_velocity_mps() {
        return get_drive_velocity().in(RadiansPerSecond) * constants.swerve.wheel_radius;
    }

    public SwerveModulePosition get_position() {
        return new SwerveModulePosition(get_distance(), get_rotation2d());
    }

    public SwerveModuleState get_state() {
        return new SwerveModuleState(get_drive_velocity_mps(), get_rotation2d()); 
    } 
}
