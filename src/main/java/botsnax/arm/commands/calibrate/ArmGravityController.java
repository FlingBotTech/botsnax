package botsnax.arm.commands.calibrate;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Preferences;
import botsnax.control.MotorController;
import botsnax.system.motor.MotorState;

import java.util.Optional;

import static edu.wpi.first.units.ImmutableMeasure.ofRelativeUnits;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static java.lang.Double.NaN;
import static java.lang.Math.cos;

public record ArmGravityController(double gain, Measure<Angle> offset) implements MotorController {
    public ArmGravityController(double gain, Measure<Angle> offset) {
        this.gain = gain;
        this.offset = ofRelativeUnits(offset.in(Degrees), Degrees);
    }

    @Override
    public double calculate(MotorState state) {
        Measure<Angle> relativeAngle = state.getAngle().minus(offset);
        return gain * cos(relativeAngle.in(Radians));
    }

    public void save(String baseName) {
        Preferences.setDouble(baseName + ".FF.kg", gain);
        Preferences.setDouble(baseName + ".FF.offsetDegrees", offset.in(Degrees));
    }

    public static Optional<ArmGravityController> load(String baseName) {
        double gain = Preferences.getDouble(baseName + ".FF.kg", NaN);
        double offsetDegrees = Preferences.getDouble(baseName + ".FF.offsetDegrees", NaN);

        if (!Double.isNaN(gain) && !Double.isNaN(offsetDegrees)) {
            return Optional.of(new ArmGravityController(gain, Degrees.of(offsetDegrees)));
        } else {
            return Optional.empty();
        }
    }
}
