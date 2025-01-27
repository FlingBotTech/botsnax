package botsnax.util;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.Radians;

public class LinearAngularConversion {
    private final double angleToDistance;

    public LinearAngularConversion(double angleToDistance) {
        this.angleToDistance = angleToDistance;
    }

    public Angle getAngle(Distance distance) {
        return Radians.of(distance.baseUnitMagnitude() / angleToDistance);
    }

    public Distance getDistance(Angle angle) {
        return BaseUnits.DistanceUnit.of(angle.in(Radians) * angleToDistance);
    }

    public AngularVelocity getAngularVelocity(LinearVelocity linearVelocity) {
        return Radians.per(BaseUnits.TimeUnit).of(linearVelocity.baseUnitMagnitude() / angleToDistance);
    }

    public LinearVelocity getLinearVelocity(AngularVelocity angularVelocity) {
        return BaseUnits.DistanceUnit.per(BaseUnits.TimeUnit).of(angularVelocity.in(Radians.per(BaseUnits.TimeUnit)) * angleToDistance);
    }

    public static LinearAngularConversion ofWheelRadiusGearRatio(Distance wheelRadius, double gearRatio) {
        return new LinearAngularConversion(wheelRadius.baseUnitMagnitude() / gearRatio);
    }
}
