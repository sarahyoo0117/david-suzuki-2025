package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class led extends SubsystemBase {
    public static final Color8Bit clr_off = new Color8Bit(0, 0, 0);
    public static final Color8Bit clr_cyan = new Color8Bit(0, 255, 255); 
    public static final Color8Bit clr_white = new Color8Bit(255, 255, 255);
    public static final Color8Bit clr_green = new Color8Bit(0, 120, 0);
    public static final Color8Bit clr_red = new Color8Bit(150, 0, 0);
    public static final Color8Bit clr_blue = new Color8Bit(0, 0, 255);
    public static final Color8Bit clr_purple = new Color8Bit(255, 0, 255);
    public static final Color8Bit clr_orange = new Color8Bit(255, 50, 0);
    public static final Color8Bit clr_yellow = new Color8Bit(255, 255, 0);

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    protected final int length;

    public led(int port, int length) {
        this.length = length;
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }

    public Command run(LEDPattern pattern) {
        return Commands.run(() -> {
            pattern.applyTo(buffer);
        }, this);
    }

    public void flag_progress(DoubleSupplier progress_supplier) {
        var pattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed);
        pattern.mask(LEDPattern.progressMaskLayer(progress_supplier));
        pattern.applyTo(buffer);
    }

    public void flag_steps(Color left_color, Color right_color, int left_start, int right_start) {
        var pattern = LEDPattern.steps(Map.of(left_start, left_color, right_start, right_color));
        pattern.applyTo(buffer); 
    }

    public void flag_solid(Color color) {
        var pattern = LEDPattern.solid(color);
        pattern.applyTo(buffer);
    }

    public void flag_rainbow() {
        var pattern = LEDPattern.rainbow(255, 128);
        var spacing = Meters.of(1 / (double)length);
        pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), spacing);
        pattern.applyTo(buffer);
    }
}
