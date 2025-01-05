package botsnax.util;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.Radians;

public class LinearAngularConversion {
    private final double angleToDistance;

    public LinearAngularConversion(double angleToDistance) {
        this.angleToDistance = angleToDistance;
    }

    public Measure<Distance> getDistance(Measure<Angle> angle) {
        return BaseUnits.Distance.of(angle.in(Radians) * angleToDistance);
    }

    public Measure<Velocity<Angle>> getAngularVelocity(Measure<Velocity<Distance>> linearVelocity) {
        return Radians.per(BaseUnits.Time).of(linearVelocity.baseUnitMagnitude() / angleToDistance);
    }

    public Measure<Velocity<Distance>> getLinearVelocity(Measure<Velocity<Angle>> angularVelocity) {
        return BaseUnits.Velocity.of(angularVelocity.in(Radians.per(BaseUnits.Time)) * angleToDistance);
    }

    public static LinearAngularConversion ofWheelRadiusGearRatio(Measure<Distance> wheelRadius, double gearRatio) {
        return new LinearAngularConversion(wheelRadius.baseUnitMagnitude() / gearRatio);
    }
}
