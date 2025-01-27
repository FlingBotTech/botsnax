package botsnax.arm.commands.calibrate;

import botsnax.control.GravityController;
import botsnax.control.MotorController;
import botsnax.system.motor.MotorState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static java.lang.Double.NaN;
import static java.lang.Math.cos;

public record ArmGravityController(Voltage gain, Angle offset) implements GravityController{
    @Override
    public Voltage calculate(MotorState state) {
        Angle relativeAngle = state.getAngle().minus(offset);
        return gain.times(cos(relativeAngle.in(Radians)));
    }

    @Override
    public Voltage getMaxFeedForward() {
        return gain;
    }

    public void save(String baseName) {
        Preferences.setDouble(baseName + ".FF.kg", gain.baseUnitMagnitude());
        Preferences.setDouble(baseName + ".FF.offsetDegrees", offset.in(Degrees));
    }

    public static Optional<ArmGravityController> load(String baseName) {
        Voltage gain = BaseUnits.VoltageUnit.of(Preferences.getDouble(baseName + ".FF.kg", NaN));
        double offsetDegrees = Preferences.getDouble(baseName + ".FF.offsetDegrees", NaN);

        if (!Double.isNaN(gain.baseUnitMagnitude()) && !Double.isNaN(offsetDegrees)) {
            return Optional.of(new ArmGravityController(gain, Degrees.of(offsetDegrees)));
        } else {
            return Optional.empty();
        }
    }
}
