package botsnax.system.motor.phoenix;

import botsnax.system.Gyro;
import botsnax.system.GyroSim;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static com.ctre.phoenix6.BaseStatusSignal.getLatencyCompensatedValue;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class PigeonGyro implements Gyro {
    private class Sim implements GyroSim {
        @Override
        public void setRawYaw(Angle angle) {
            pigeon.getSimState().setRawYaw(angle);
        }

        @Override
        public void setAngularVelocityZ(AngularVelocity angularVelocity) {
            pigeon.getSimState().setAngularVelocityZ(angularVelocity);
        }
    }

    private final Pigeon2 pigeon;
    private final GyroSim sim;

    public PigeonGyro(Pigeon2 pigeon) {
        this.pigeon = pigeon;
        this.sim = isSimulation() ? new Sim() : null;
    }

    public Pigeon2 get() {
        return pigeon;
    }

    @Override
    public GyroSim getSim() {
        return sim;
    }

    @Override
    public Rotation2d getHeading() {
        return fromRadians(getLatencyCompensatedValue(pigeon.getYaw(), pigeon.getAngularVelocityZWorld()).in(Radians));
    }
}
