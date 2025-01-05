package botsnax.system.motor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.flywheel.Flywheel;

public interface VelocitySetter {
    void apply(Measure<Velocity<Angle>> velocity, Flywheel flywheel);
}
