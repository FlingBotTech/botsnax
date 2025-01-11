package botsnax.system;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
    GyroSim getSim();
    Rotation2d getHeading();
}
