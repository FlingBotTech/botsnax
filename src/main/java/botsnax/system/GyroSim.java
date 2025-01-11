package botsnax.system;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroSim {
    void setRawYaw(Angle angle);
    void setAngularVelocityZ(AngularVelocity angularVelocity);
}
