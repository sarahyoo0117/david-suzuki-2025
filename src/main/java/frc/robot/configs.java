package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;

public class configs {
    public static final String canbus = "canbus";        

    public final class swerve {
        public static final class module_config {
            public final int drive_id;
            public final int steer_id;
            public final Translation2d module_offset;
            public final InvertedValue drive_inverted;
            public final InvertedValue steer_inverted;
            public final int abs_channel;
            public final boolean abs_inverted;
            public final double abs_offset;

            public module_config(int drive_id, int steer_id, Translation2d module_offset, InvertedValue drive_inverted, InvertedValue steer_inverted, 
                int abs_channel, boolean abs_inverted, double abs_offset) {
                this.drive_id = drive_id;
                this.steer_id = steer_id;
                this.module_offset = module_offset;
                this.drive_inverted = drive_inverted;
                this.steer_inverted = steer_inverted;
                this.abs_channel = abs_channel;
                this.abs_offset = abs_offset;
                this.abs_inverted = abs_inverted;
            }
        }

        public static final module_config[] module_configs = {
            new module_config(constants.ids.fr_drive, constants.ids.fr_steer, constants.swerve.module_offsets[0], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, constants.ids.fr_abs, true, 0),
            new module_config(constants.ids.fl_drive, constants.ids.fl_steer, constants.swerve.module_offsets[1], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, constants.ids.fl_abs, true, 0),
            new module_config(constants.ids.br_drive, constants.ids.br_steer, constants.swerve.module_offsets[2], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, constants.ids.br_abs, true, 0),
            new module_config(constants.ids.bl_drive, constants.ids.bl_steer, constants.swerve.module_offsets[3], 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, constants.ids.bl_abs, true, 0)
        };

        public static final TalonFXConfiguration drive_config(InvertedValue inverted) {
            return new TalonFXConfiguration()
                .withSlot0(
                    new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKV(0)
                    .withKS(0)
                    .withKG(0)
                    .withKA(0)
                )
                .withMotorOutput(
                    new MotorOutputConfigs()
                    .withInverted(inverted)
                );
        }

        public static final TalonFXConfiguration steer_config(InvertedValue inverted) {
            return new TalonFXConfiguration()
                .withSlot0(
                    new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKV(0)
                    .withKS(0)
                    .withKG(0)
                    .withKA(0)
                )
                .withMotorOutput(
                    new MotorOutputConfigs()
                    .withInverted(inverted)
                );
        } 
    }  
}
