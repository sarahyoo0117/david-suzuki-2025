package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.tags;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configs.LL;
import frc.robot.constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.math_utils;

public class swerve extends swerve_lowlevel {
    //TODO: create custom pid motor class with simple feed forward
    PIDController x_pid = new PIDController(2.0, 0, 0);
    PIDController y_pid = new PIDController(2.0, 0, 0);
    PIDController turn_pid = new PIDController(2.0, 0, 0);

    private int reef_assist_tag_id = 0; 

    @Override
    public void periodic() {
        catch_reef_id();
    }

    public void catch_reef_id() {
        Translation2d pos = get_pose2d().getTranslation();
        boolean is_red = get_pose2d().getX() > constants.tags.getFieldLength() / 2.0;
        int[] tag_ids = (is_red) ? constants.red_reef_tag_ids : constants.blue_reef_tag_ids;
        double closest_dist = Double.MAX_VALUE;
        for (int id : tag_ids) {
            Pose3d tag = tags.getTagPose(id).get();
            Translation2d tag_pos = tag.getTranslation().toTranslation2d();
            double dist = pos.getDistance(tag_pos); 
            if (closest_dist > dist) {
                closest_dist = dist;
                reef_assist_tag_id = id;
            }
        }
    }

    public boolean is_close_to_reef() {
        Pose3d tag = tags.getTagPose(reef_assist_tag_id).get();
        Translation2d tag_pos = tag.getTranslation().toTranslation2d();
        Rotation2d tag_angle = Rotation2d.fromRadians(tag.getRotation().getMeasureZ().in(Radians));
        Translation2d tag_offset = math_utils.find_trans2d(tag_angle, 0.5);
        return tag_pos.plus(tag_offset).getDistance(get_pose2d().getTranslation()) < 0.8;
    }

    //TODO: auto alginment
    public Command auto_align() {
        Translation2d tag_pos = tags.getTagPose(reef_assist_tag_id).get().getTranslation().toTranslation2d();
        return strafe_to_point(tag_pos, 2.0, 1.0) 
        .until(() -> is_close_to_reef());
    }

    public Command strafe_to_point(Translation2d point, double max_speed, double tolerance) {
        return set_speeds(() -> {
            Translation2d error = point.minus(get_pose2d().getTranslation());
            double error_len = error.getNorm();
            double x_speed = MathUtil.clamp(x_pid.calculate(error_len), -max_speed, max_speed);
            if (x_speed < 0.02) {
                x_speed = 0;
            }
            Translation2d output = error.div((error_len == 0) ? 1 : error_len).times(x_speed);
            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(output.getX(), output.getY(), 0), get_heading());
        })
        .until(() -> point.minus(get_pose2d().getTranslation()).getNorm() <= tolerance);
    }

    public Command strafe_to_visible_tag(LL ll, double max_speed, double tolerance) {
        if (LimelightHelpers.getTV(ll.name)) {
            Translation2d tag_point = math_utils.tag_trans2d(ll);
            return strafe_to_point(tag_point, max_speed, tolerance);
        }
        return Commands.none();
    }
}
