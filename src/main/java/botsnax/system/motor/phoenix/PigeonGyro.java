package botsnax.system.motor.phoenix;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import botsnax.system.Gyro;

import static com.ctre.phoenix6.BaseStatusSignal.getLatencyCompensatedValue;
import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

public class PigeonGyro implements Gyro {
    private Pigeon2 pigeon;

    public PigeonGyro(Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    public Pigeon2 get() {
        return pigeon;
    }

    @Override
    public Rotation2d getHeading() {
        return fromDegrees(getLatencyCompensatedValue(pigeon.getYaw(), pigeon.getAngularVelocityZWorld()));
    }
}
