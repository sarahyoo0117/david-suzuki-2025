package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configs.LL;
import frc.robot.utils.LimelightHelpers;

public class swerve extends swerve_lowlevel {
    //TODO: create custom pid motor class with simple feed forward
    PIDController x_pid = new PIDController(2.0, 0, 0);
    PIDController y_pid = new PIDController(2.0, 0, 0);
    PIDController turn_pid = new PIDController(2.0, 0, 0);
    SimpleMotorFeedforward x_ff = new SimpleMotorFeedforward(0, 0);
    SimpleMotorFeedforward y_ff = new SimpleMotorFeedforward(0, 0);
    SimpleMotorFeedforward turn_ff = new SimpleMotorFeedforward(0, 0);

    //TODO: max speed, tolerance
    public Command strafe_to_tag(LL ll, double max_speed, double tolerance) {
        //every angles is in degrees
        return set_speeds(() -> {
            if (LimelightHelpers.getTV(ll.name)) {
                Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(ll.name);
                double norm = pose.getTranslation().getNorm();
                double ty =  LimelightHelpers.getTY(ll.name) + ll.mounted_angle.getDegrees();
                double tx = LimelightHelpers.getTX(ll.name);
                //x axis is vertical, y axis is horizontal
                double x_dist = Math.cos(ty) * norm;
                double y_dist = Math.tan(tx) * x_dist;
                Translation2d tag_point = new Translation2d(x_dist, y_dist);

                //output calculation
                Translation2d error = get_pose2d().getTranslation().minus(tag_point);
                double error_len = error.getNorm();
                //unit speed to reach the tag point
                double x_speed = MathUtil.clamp(x_pid.calculate(error_len), -max_speed, max_speed);
                //pid returns 1d unit vector for speed output, so this calculates the total output
                Translation2d output = error.div((error_len == 0) ? 1 : error_len).times(x_speed);
                return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(output.getX(), output.getY(), 0), get_heading());
            }
            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(), get_heading());
        });
    }
}
