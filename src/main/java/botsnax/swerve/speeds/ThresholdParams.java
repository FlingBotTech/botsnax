package botsnax.swerve.speeds;

import botsnax.vision.PoseStdDev;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Radians;

public record ThresholdParams(
        LinearVelocity minSpeed,
        Time systemLatency,
        PoseStdDev poseStdDev,
        Time sensorLatency,
        Distance centerToWheelDistance
) {
    AngularVelocity minAngularVelocity() {
        return getAngularVelocity(minSpeed);
    }

    public Distance getThreshold() {
        return minSpeed.times(systemLatency.plus(sensorLatency)).plus(poseStdDev.getRadiusStdDev().times(6));
    }

    public Angle getAngularThreshold() {
        return minAngularVelocity().times(systemLatency.plus(sensorLatency)).plus(poseStdDev.rStdDev().times(0.125));
    }

    public AngularVelocity getAngularVelocity(LinearVelocity linearVelocity) {
        return Radians.per(BaseUnits.TimeUnit).of(linearVelocity.baseUnitMagnitude() / centerToWheelDistance.baseUnitMagnitude());
    }
}
