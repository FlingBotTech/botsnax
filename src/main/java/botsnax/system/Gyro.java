package botsnax.system;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface Gyro {
    GyroSim getSim();
    Rotation2d getHeading();
    AngularVelocity getVelocity();
}
