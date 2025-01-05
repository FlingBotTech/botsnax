package botsnax.system.encoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface Encoder {
    Measure<Angle> getAngle();
    Measure<Velocity<Angle>> getVelocity();

    void setAngle(Measure<Angle> angle);
}
