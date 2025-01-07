package botsnax.system.motor;

import botsnax.flywheel.Flywheel;
import edu.wpi.first.units.measure.AngularVelocity;

public interface VelocitySetter {
    void apply(AngularVelocity velocity, Flywheel flywheel);
}
