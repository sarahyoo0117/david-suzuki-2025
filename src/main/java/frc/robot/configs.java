package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class configs {
    public static final class LL {
        public final String name;
        public final Translation3d mounted_offset;
        public final Rotation2d mounted_angle;
        public LL(String name, Rotation2d mounted_angle, Translation3d mounted_offset) {
            this.name = name;
            this.mounted_angle = mounted_angle;
            this.mounted_offset = mounted_offset;
        }
    }

    public static final class can {
        public String canbus;    
        public int id;
        public can(int id, String canbus) {
            this.id = id;
            this.canbus = canbus;
        } 
    }

    public static final double pole_spacing_m = Units.inchesToMeters(13);
    public static final double ll_up_m = Units.inchesToMeters(6.75);
    public static final double ll_forward_m = Units.inchesToMeters(10);
    public static final Rotation2d ll_mounted_angle = Rotation2d.fromDegrees(31);
    public static final LL ll_left = new LL("limelight-left", ll_mounted_angle, new Translation3d(ll_forward_m, pole_spacing_m/2.0, ll_up_m));
    public static final LL ll_right = new LL("limelight-right", ll_mounted_angle, new Translation3d(ll_forward_m, -pole_spacing_m/2.0, ll_up_m));

    public static final String canbus = "canivore";        
    public static final String canbus_rio = "rio";

    public static final can can_swerve_fr_drive = new can(8, canbus); 
    public static final can can_swerve_fr_turn = new can(9, canbus); 
    public static final int dio_swerve_fr_abs = 7;

    public static final can can_swerve_fl_drive = new can(6, canbus); 
    public static final can can_swerve_fl_turn = new can(3, canbus); 
    public static final int dio_swerve_fl_abs = 6;

    public static final can can_swerve_br_drive = new can(2, canbus); 
    public static final can can_swerve_br_turn = new can(4, canbus); 
    public static final int dio_swerve_br_abs = 9;

    public static final can can_swerve_bl_drive = new can(7, canbus); 
    public static final can can_swerve_bl_turn = new can(5, canbus); 
    public static final int dio_swerve_bl_abs = 8;
    
    public static final can can_pigeon = new can(10, canbus);

    public static final can can_elevator_motor_left = new can(12, canbus);
    public static final can can_elevator_motor_right = new can(13, canbus);

    public static final can can_end_effector_roller = new can(16, canbus_rio);
    public static final can can_end_effector_pivot = new can(15, canbus_rio);
    public static final int end_effector_lidar = 5;

    public static final can can_ramp_roller = new can(14, canbus_rio);
    public static final int ramp_lidar_edge = 4;
    public static final int ramp_lidar_middle = 3;

    public final class swerve {
        public static final class module_config {
            public final can can_drive;
            public final can can_turn;
            public final Translation2d module_offset;
            public final InvertedValue drive_inverted;
            public final InvertedValue turn_inverted;
            public final int abs_channel;
            public final boolean abs_inverted;
            public final double abs_offset;

            public module_config(can can_drive, can can_turn, Translation2d module_offset, InvertedValue drive_inverted, InvertedValue turn_inverted, 
                int abs_channel, boolean abs_inverted, double abs_offset) {
                this.can_drive = can_drive;
                this.can_turn = can_turn;
                this.module_offset = module_offset;
                this.drive_inverted = drive_inverted;
                this.turn_inverted = turn_inverted;
                this.abs_channel = abs_channel;
                this.abs_offset = abs_offset;
                this.abs_inverted = abs_inverted;
            }
        }
        //abs in rotations or radians?!
        public static final module_config[] module_configs = {
            new module_config(can_swerve_fr_drive, can_swerve_fr_turn, constants.swerve.module_offsets[0], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, dio_swerve_fr_abs, false, 0.275),
            new module_config(can_swerve_fl_drive, can_swerve_fl_turn, constants.swerve.module_offsets[1], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, dio_swerve_fl_abs, false, 0.432),
            new module_config(can_swerve_br_drive, can_swerve_br_turn, constants.swerve.module_offsets[2], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, dio_swerve_br_abs, false, 0.063),
            new module_config(can_swerve_bl_drive, can_swerve_bl_turn, constants.swerve.module_offsets[3], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, dio_swerve_bl_abs, false, 0.922)
        };

        public static final TalonFXConfiguration drive_config(InvertedValue inverted) {
            return new TalonFXConfiguration()
                .withSlot0(
                    new Slot0Configs()
                    .withKP(0.0)
                    .withKV(0.4)
                )
                .withMotorOutput(
                    new MotorOutputConfigs()
                    .withInverted(inverted)
                    .withDutyCycleNeutralDeadband(0.05)
                    .withNeutralMode(NeutralModeValue.Brake)
                )
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false) 
                );
        }

        public static final TalonFXConfiguration turn_config(InvertedValue inverted) {
            return new TalonFXConfiguration()
                .withSlot0(
                    new Slot0Configs()
                    .withKP(50)
                    .withKI(1.5)
                    .withKV(0.15)
                )
                .withMotorOutput(
                    new MotorOutputConfigs()
                    .withInverted(inverted)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(150.0/7.0)
                )
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(true)
                );
        } 
    }  

    public final class elevtor {
        public static final TalonFXConfiguration elevator_left_config() {
            return new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                    .withKP(50)
                    .withKV(1.5)
                )
                .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(4))
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(4))
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(17.6)
                )
                .withMotorOutput(
                    new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)       
                );
        }

        public static final TalonFXConfiguration elevator_right_config() {
            return new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                    .withKP(50)
                    .withKV(1.5)
                )
                .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(4))
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(4))
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(17.6)
                )
                .withMotorOutput(
                    new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)       
                );
        }
    }

    public final class end_effector {
        public static final TalonFXConfiguration roller_config() {
            return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.06)
                .withKV(0.09)
            );
        }
    }
}
