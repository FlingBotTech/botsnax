package botsnax.system.motor;

import botsnax.flywheel.Flywheel;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public interface VelocitySetter {
    void apply(AngularVelocity velocity, Flywheel flywheel);

    static VelocitySetter createOpenLoop(AngularVelocity speedAtNominalVoltage) {
        return (velocity, flywheel) -> {
            Voltage voltage = Volts.of(12).times(velocity.baseUnitMagnitude() / speedAtNominalVoltage.baseUnitMagnitude());
            flywheel.setVoltage(voltage);
        };
    }
}
