package botsnax.system.motor.phoenix;

import botsnax.system.Gyro;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

import static com.ctre.phoenix6.BaseStatusSignal.getLatencyCompensatedValue;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Radians;

public class PigeonGyro implements Gyro {
    private final Pigeon2 pigeon;

    public PigeonGyro(Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    public Pigeon2 get() {
        return pigeon;
    }

    @Override
    public Rotation2d getHeading() {
        return fromRadians(getLatencyCompensatedValue(pigeon.getYaw(), pigeon.getAngularVelocityZWorld()).in(Radians));
    }
}
