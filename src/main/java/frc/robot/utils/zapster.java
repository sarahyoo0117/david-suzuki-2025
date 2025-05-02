package frc.robot.utils;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class zapster extends TalonFX {
    private DCMotorSim sim; 
    private double dts;
    private StatusSignal<Voltage> volt;

    private final PIDController[] sim_pids = {
        new PIDController(0, 0, 0), //pid for x pos
        new PIDController(0, 0, 0), //pid for y pos
        new PIDController(0, 0, 0), //pid for velocity
    };
    private final SimpleMotorFeedforward[] sim_ffs = {
        new SimpleMotorFeedforward(0, 0), //ff for x pid
        new SimpleMotorFeedforward(0, 0), //ff for y pid
        new SimpleMotorFeedforward(0, 0), //ff for velocity pid
    };
    
    public zapster(int id, String canbus) {
        super(id, canbus);
    }

    private void sim_periodic() {
        volt = getMotorVoltage(true);
        sim.setInputVoltage(volt.getValue().in(Volt));
        sim.update(dts);
    }
    
    public zapster make_sim(TimedRobot robot, double hertz, DCMotor motor, double gear_ratio, double moment_of_inertia) {
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moment_of_inertia, gear_ratio), motor);
        dts = 1 / hertz; 
        sim_pids[0] = new PIDController(0, 0, 0, dts);
        sim_pids[1] = new PIDController(0, 0, 0, dts);
        sim_pids[2] = new PIDController(0, 0, 0, dts);
        robot.addPeriodic(this::sim_periodic, dts);
        return this;
    }

    public zapster with_config(TalonFXConfiguration config) {
       sim_pids[0].setPID(config.Slot0.kP, config.Slot0.kI, config.Slot0.kD); 
       sim_ffs[0] = new SimpleMotorFeedforward(config.Slot0.kS, config.Slot0.kV);
       sim_pids[1].setPID(config.Slot1.kP, config.Slot1.kI, config.Slot1.kD); 
       sim_ffs[1] = new SimpleMotorFeedforward(config.Slot1.kS, config.Slot1.kV);
       sim_pids[2].setPID(config.Slot2.kP, config.Slot2.kI, config.Slot2.kD); 
       sim_ffs[2] = new SimpleMotorFeedforward(config.Slot2.kS, config.Slot2.kV);
       return this;
    }
}
