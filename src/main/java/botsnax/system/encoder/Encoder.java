package botsnax.system.encoder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;

public interface Encoder {
    Angle getAngle();
    AngularVelocity getVelocity();

    void setAngle(Angle angle);
}
