package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configs.LL;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.math_utils;

public class swerve extends swerve_lowlevel {
    //TODO: create custom pid motor class with simple feed forward
    PIDController x_pid = new PIDController(2.0, 0, 0);
    PIDController y_pid = new PIDController(2.0, 0, 0);
    PIDController turn_pid = new PIDController(2.0, 0, 0);

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

    public Command strafe_to_tag(LL ll, double max_speed, double tolerance) {
        if (LimelightHelpers.getTV(ll.name)) {
            Translation2d tag_point = math_utils.tag_dist2d(ll);
            return strafe_to_point(tag_point, max_speed, tolerance);
        }
        return Commands.none();
    }
    //TODO: check if close enough to reef
    //TODO: auto alginment
}
