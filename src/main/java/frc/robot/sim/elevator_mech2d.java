package frc.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class elevator_mech2d {
    private final Mechanism2d mech;
    private MechanismLigament2d elevator;
    private MechanismLigament2d end_effector;

    public elevator_mech2d(int width, int height) {
        mech = new Mechanism2d(width, height);
        MechanismRoot2d root = mech.getRoot("elevator", width/2, height/2);
        elevator = root.append(new MechanismLigament2d("desired_elevator_pos", 1, 0));
        end_effector = root.append(new MechanismLigament2d("desired_endeff_pos", 1, 0));
        end_effector.setColor(new Color8Bit(Color.kAqua));
        SmartDashboard.putData("elevator_mech2d", mech);
    }

    public void set_elevator_pos(double degrees) {
        elevator.setAngle(degrees);
    }
}
