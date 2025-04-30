package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;
import frc.robot.constants;
import frc.robot.sim.swerve_mech2d;

public class swerve extends SubsystemBase{
    private final swerve_module[] modules = new swerve_module[4];
    private final Pigeon2 pig = new Pigeon2(configs.ids.pigeon, configs.canbus);
    private final SwerveDrivePoseEstimator pose_estimator; 
    private final Field2d field = new Field2d();
    private final swerve_mech2d mech = new swerve_mech2d(3);
    private ChassisSpeeds desired = new ChassisSpeeds();
    
    public swerve() {
        modules[0] = new swerve_module(configs.swerve.module_configs[0]);
        modules[1] = new swerve_module(configs.swerve.module_configs[1]);
        modules[2] = new swerve_module(configs.swerve.module_configs[2]);
        modules[3] = new swerve_module(configs.swerve.module_configs[3]);
        pose_estimator = new SwerveDrivePoseEstimator(constants.swerve.drive_kinematics, get_heading(), get_module_positions(), new Pose2d());
        mech.init();
        SmartDashboard.putData("field2d", field);
        SmartDashboard.putNumberArray("modules_raw_abs", new double[] { 
            modules[0].get_abs_raw(), 
            modules[1].get_abs_raw(),
            modules[2].get_abs_raw(),
            modules[3].get_abs_raw(),
        });
    } 

    @Override
    public void periodic() {
        SwerveModuleState[] desired_states = constants.swerve.drive_kinematics.toSwerveModuleStates(desired);
        apply_module_states(desired_states);
        pig.getSimState();
        mech.update(get_heading(), desired_states, get_modules_states());
        field.setRobotPose(get_pose2d());
    }
    
    public void set_desired_speeds(double x_speed, double y_speed, double omega) {
        desired.vxMetersPerSecond = x_speed;
        desired.vyMetersPerSecond = y_speed;
        desired.omegaRadiansPerSecond = omega;
    } 

    public Command set_speeds(Supplier<ChassisSpeeds> chassis_supplier) {
        ChassisSpeeds speeds = chassis_supplier.get();
        return Commands.run(() -> {
            set_desired_speeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }, this);
    }

    public void reset_heading() {
        pig.reset();
    }
    
    public Rotation2d get_heading() {
        return pig.getRotation2d();
    }

    public Pose2d get_pose2d() {
        if (RobotBase.isReal()) {
            return pose_estimator.update(get_heading(), get_module_positions());
        }
        return new Pose2d(new Translation2d(desired.vxMetersPerSecond, desired.vyMetersPerSecond), get_heading());
    } 

    public SwerveModuleState[] get_modules_states() {
        return new SwerveModuleState[] {
            modules[0].get_state(),
            modules[1].get_state(),
            modules[2].get_state(),
            modules[3].get_state()
        };
    }

    public void apply_module_states(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, constants.swerve.max_module_speed_mps);
        modules[0].apply_state(states[0]);
        modules[1].apply_state(states[1]);
        modules[2].apply_state(states[2]);
        modules[3].apply_state(states[3]);
    }

    public SwerveModulePosition[] get_module_positions() {
        return new SwerveModulePosition[] {
           modules[0].get_position(),
           modules[1].get_position(),
           modules[2].get_position(),
           modules[3].get_position(),
        };
    }
}
