package botsnax.elevator.calibrate;

import botsnax.control.GravityController;
import botsnax.system.motor.MotorState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

import java.util.Optional;

import static java.lang.Double.NaN;

public record ElevatorGravityController(Voltage gravityVoltage) implements GravityController {
    @Override
    public Voltage calculate(MotorState state) {
        return gravityVoltage;
    }

    @Override
    public Voltage getMaxFeedForward() {
        return gravityVoltage;
    }

    public void save(String baseName) {
        Preferences.setDouble(baseName + ".FF.kg", gravityVoltage.baseUnitMagnitude());
    }

    public static Optional<ElevatorGravityController> load(String baseName) {
        Voltage gain = BaseUnits.VoltageUnit.of(Preferences.getDouble(baseName + ".FF.kg", NaN));

        if (!Double.isNaN(gain.baseUnitMagnitude())) {
            return Optional.of(new ElevatorGravityController(gain));
        } else {
            return Optional.empty();
        }
    }
}
