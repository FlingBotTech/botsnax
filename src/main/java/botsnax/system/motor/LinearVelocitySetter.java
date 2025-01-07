package botsnax.system.motor;

import botsnax.flywheel.Flywheel;
import botsnax.util.LinearAngularConversion;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public interface LinearVelocitySetter {
    void setVelocity(LinearVelocity linearVelocity, Flywheel flywheel);

    static LinearVelocitySetter createOpenLoop(LinearVelocity speedAtNominalVoltage) {
        return (linearVelocity, flywheel) -> {
            Voltage voltage = Volts.of(12).times(linearVelocity.baseUnitMagnitude() / speedAtNominalVoltage.baseUnitMagnitude());
            flywheel.setVoltage(voltage);
        };
    }

    static LinearVelocitySetter createClosedLoop(LinearAngularConversion conversion, VelocitySetter velocitySetter) {
        return (linearVelocity, flywheel) -> velocitySetter.apply(
                conversion.getAngularVelocity(linearVelocity),
                flywheel);
    }
}
