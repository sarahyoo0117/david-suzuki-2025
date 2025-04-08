package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class constants {
    public final class ids{
        //TODO: set CAN device ids
        public static final int fr_drive = 0;
        public static final int fr_turn = 0;
        public static final int fr_abs = 0;

        public static final int fl_drive = 0;
        public static final int fl_turn = 0;
        public static final int fl_abs = 1;

        public static final int br_drive = 0;
        public static final int br_turn = 0;
        public static final int br_abs = 2;

        public static final int bl_drive = 0;
        public static final int bl_turn = 0;
        public static final int bl_abs = 3;

        public static final int pigeon = 0;

        public static final int elevator_motor_left = 0;
        public static final int elevator_motor_right = 0;
        public static final int elevator_homming_sensor = 0;
    }
    
    public final class swerve {
        public static final double wheel_diameter = Units.inchesToMeters(3.75);
        public static final double wheel_radius = wheel_diameter / 2;
        public static final double half_wheelbase_meters = Units.inchesToMeters(26) / 2;
        //L3 swerve
        public static final double max_module_speed_mps = FeetPerSecond.of(16.5).in(MetersPerSecond);

        public static final Translation2d offset_fr = new Translation2d(half_wheelbase_meters, half_wheelbase_meters);
        public static final Translation2d offset_fl = new Translation2d(half_wheelbase_meters, -half_wheelbase_meters);
        public static final Translation2d offset_br = new Translation2d(-half_wheelbase_meters, half_wheelbase_meters);
        public static final Translation2d offset_bl = new Translation2d(-half_wheelbase_meters, -half_wheelbase_meters);
        
        public static final Translation2d[] module_offsets = { offset_fr, offset_fl, offset_br, offset_bl };
    
        public static final SwerveDriveKinematics drive_kinematics = new SwerveDriveKinematics(
            module_offsets[0],
            module_offsets[1],
            module_offsets[2],
            module_offsets[3]
        );
    }

    public final class elevator {
        //TODO: set heights for elevator states 
        public static enum elevator_state {
            HOME(0.5),
            L1(1.0),
            L2(2.0),
            L3(3.0),
            L4(4.0),
            ALGAE_INTAKE1(2.0),
            ALGAE_INTAKE2(2.0),
            ALGAE_NET(4.0),
            ALGAE_PROCESSOR(1.0);
            public final double height;
            private elevator_state(double height) {
                this.height = height;
            }
        }
    }
}
