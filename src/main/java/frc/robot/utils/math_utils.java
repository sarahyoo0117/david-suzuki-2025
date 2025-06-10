package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.configs.LL;

public class math_utils {

    public static Translation2d tag_dist2d(LL ll) {
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
}
