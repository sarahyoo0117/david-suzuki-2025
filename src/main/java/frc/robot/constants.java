package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class constants {
    public static final AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final int[] red_reef_tag_ids = {10, 9, 8, 7, 6, 11}; //in clock-wise order from 10
    public static final int[] blue_reef_tag_ids = {21, 20, 19, 18, 17, 22}; //in counter-clockwise order from 21

    public final class elevator {
        public static enum elevator_state {
            MANUAL(Meters.of(0)),
            HOME(Meters.of(0.03)),
            L1(Meters.of(0.48)),
            L2(Inches.of(0.8)),
            L3(Meters.of(1.2)),
            //TODO: set heights for elevator states 
            L4(Meters.of(1.5)),
            ALGAE_REEF1(Meters.of(0.37)),
            ALGAE_REEF2(Meters.of(0.85)),
            ALGAE_NET(Meters.of(1.2)),
            ALGAE_PROCESSOR(Meters.of(0.0));
            public final Distance height;
            private elevator_state(Distance height) {
                this.height = height;
            }
        }
    }

    public final class end_effector {
        public static final AngularVelocity intake_coral = RotationsPerSecond.of(-18); 
        public static final AngularVelocity intake_algae = RotationsPerSecond.of(20); 
        public static final AngularVelocity spit_algae = RotationsPerSecond.of(-30);
        public static final AngularVelocity spit_coral = RotationsPerSecond.of(30);
        public static final Angle pivot_score_coral_L1 = Degrees.of(-28); 
        public static final Angle pivot_intake_algae = Degrees.of(21); 
        public static final Angle pivot_zero = Degrees.of(100); 
        public static final Angle pivot_fully_down = Degrees.of(0); 
        public static final Current hold_algae_current = Amps.of(50);
    }

    public final class ramp {
        public static final AngularVelocity intake_coral = RotationsPerSecond.of(18); 
        public static final AngularVelocity unjam_coral = RotationsPerSecond.of(-5); 
    }

    public final class swerve {
        public static final double wheel_diameter = Units.inchesToMeters(3.75);
        public static final double wheel_radius = wheel_diameter / 2;
        public static final double half_wheelbase_meters = Units.inchesToMeters(26) / 2;
        //L3 swerve
        public static final double max_module_speed_mps = FeetPerSecond.of(16.5).in(MetersPerSecond);

        //y-axis is horizontal, x-axis is vertical
        public static final Translation2d offset_fr = new Translation2d(half_wheelbase_meters, -half_wheelbase_meters);
        public static final Translation2d offset_fl = new Translation2d(half_wheelbase_meters, half_wheelbase_meters);
        public static final Translation2d offset_br = new Translation2d(-half_wheelbase_meters, -half_wheelbase_meters);
        public static final Translation2d offset_bl = new Translation2d(-half_wheelbase_meters, half_wheelbase_meters);
        
        public static final Translation2d[] module_offsets = { offset_fr, offset_fl, offset_br, offset_bl };
    
        public static final SwerveDriveKinematics drive_kinematics = new SwerveDriveKinematics(
            module_offsets[0],
            module_offsets[1],
            module_offsets[2],
            module_offsets[3]
        );
    }
}
