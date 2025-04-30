package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class elevator_mech2d {
    private final Mechanism2d mech;
    private MechanismLigament2d target_height, actual_height, target_wrist, actual_wrist;

    public elevator_mech2d(int width, int height) {
        mech = new Mechanism2d(width, height);
        MechanismRoot2d root = mech.getRoot("elevator", width/2, height/2);
        target_height = root.append(new MechanismLigament2d("elevator_target_height", 1, 90));
        target_wrist = target_height.append(new MechanismLigament2d("target_wrist", 1, -90));
        target_height.setColor(new Color8Bit(Color.kAqua));
        target_wrist.setColor(new Color8Bit(Color.kAqua));
        actual_height = root.append(new MechanismLigament2d("elevator_actual_height", 1, 90));
        actual_wrist = actual_height.append(new MechanismLigament2d("actual_wrist", 1, -90));
        SmartDashboard.putData("elevator_mech2d", mech);
    }

    public void update_elevator_pos(Distance target, Distance actual) {
        target_height.setLength(target.in(Meters));
        actual_height.setLength(actual.in(Meters));
    }

    public void update_wrist_angle(Angle target, Angle actual) {
        target_wrist.setAngle(target.in(Degrees));
        actual_wrist.setAngle(actual.in(Degrees));
    } 
}
