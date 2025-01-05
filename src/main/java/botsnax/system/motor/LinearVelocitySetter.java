package botsnax.system.motor;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.flywheel.Flywheel;
import botsnax.util.LinearAngularConversion;

public interface LinearVelocitySetter {
    void setVelocity(Measure<Velocity<Distance>> linearVelocity, Flywheel flywheel);

    static LinearVelocitySetter createOpenLoop(Measure<Velocity<Distance>> speedAtNominalVoltage) {
        return (linearVelocity, flywheel) -> {
            double voltage = linearVelocity.baseUnitMagnitude() / speedAtNominalVoltage.baseUnitMagnitude() * 12;
            flywheel.setVoltage(voltage);
        };
    }

    static LinearVelocitySetter createClosedLoop(LinearAngularConversion conversion, VelocitySetter velocitySetter) {
        return (linearVelocity, flywheel) -> velocitySetter.apply(
                conversion.getAngularVelocity(linearVelocity),
                flywheel);
    }
}
