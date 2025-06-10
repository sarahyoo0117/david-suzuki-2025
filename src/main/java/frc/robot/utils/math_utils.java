package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.configs.LL;

public class math_utils {
    //norm: vector distance from the origin to a point
    //distance: distance between two points

    public static Translation2d tag_trans2d(LL ll) {
        if (!LimelightHelpers.getTV(ll.name)) {
            return new Translation2d();
        }
        Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(ll.name);
        double norm = pose.getTranslation().getNorm();
        double ty =  LimelightHelpers.getTY(ll.name) + ll.mounted_angle.getDegrees();
        double tx = LimelightHelpers.getTX(ll.name);
        //x axis is vertical, y axis is horizontal
        double x_dist = Math.cos(ty) * norm;
        double y_dist = Math.tan(tx) * x_dist;
        return new Translation2d(x_dist, y_dist);
    }

    public static Translation2d find_trans2d(Rotation2d angle, double hypot_m) {
        return new Translation2d(angle.getCos() * hypot_m, angle.getSin() * hypot_m);
    }
}
