package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.configs;
import frc.robot.constants;
import frc.robot.configs.swerve.module_config;

public class swerve_module {
    private final module_config module_config; 
    private TalonFX drive;
    private TalonFX steer; 
    private DutyCycleEncoder abs;

    public swerve_module(module_config module_config) {
        this.module_config = module_config;
        drive = new TalonFX(module_config.drive_id, configs.canbus);
        steer = new TalonFX(module_config.steer_id, configs.canbus);
        drive.getConfigurator().apply(configs.swerve.drive_config(module_config.drive_inverted));
        steer.getConfigurator().apply(configs.swerve.steer_config(module_config.steer_inverted));
        abs = new DutyCycleEncoder(module_config.abs_channel);
        abs.setDutyCycleRange(1 / 4096, 4096 / 4096);
        abs.setInverted(module_config.abs_inverted);
    }

    public void apply_state(SwerveModuleState state) {
        state.optimize(get_rotation2d());
        //TODO: drive and steer voltage
        drive.setControl(new VelocityVoltage(Units.radiansToRotations(state.speedMetersPerSecond / constants.wheel_radius)));
        steer.setControl(new PositionVoltage(state.angle.getRotations()));
    } 

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public Distance get_distance() { 
        return Meters.of(constants.wheel_radius).times(drive.getPosition().getValue().in(Radians)); 
    }

    public Rotation2d get_rotation2d() {
        return Rotation2d.fromRadians(steer.getPosition().getValue().in(Radians));
    } 

    public SwerveModulePosition get_position() {
        return new SwerveModulePosition(get_distance(), get_rotation2d());
    }
}
