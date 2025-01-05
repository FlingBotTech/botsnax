package botsnax.system.motor;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AngleSetter {
    void apply(Rotation2d angle, MotorSystem motor);
}
