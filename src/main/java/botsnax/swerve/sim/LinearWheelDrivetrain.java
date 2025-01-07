package botsnax.swerve.sim;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.pow;

public record LinearWheelDrivetrain(Wheels wheels, Mass carriageMass, double frictionCoefficient) {
    private Distance convertPositionTo(Angle angle) {
        return wheels.radius().times(angle.in(Radians));
    }

    public AngularVelocity convertVelocityFrom(LinearVelocity velocity) {
        return BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(velocity.baseUnitMagnitude() / wheels.radius().baseUnitMagnitude());
    }

    private LinearVelocity convertVelocityTo(AngularVelocity velocity) {
        return BaseUnits.DistanceUnit.per(BaseUnits.TimeUnit).of(velocity.baseUnitMagnitude() * wheels.radius().baseUnitMagnitude());
    }

    private MomentOfInertia getMoment() {
        return KilogramSquareMeters.of(carriageMass.in(Kilograms) * pow(wheels.radius().in(Meters), 2));
    }

    public LinearVelocity getMaxSpeed() {
        return convertVelocityTo(wheels.getMaxSpeed());
    }

    public LinearVelocity getVelocityChange(LinearVelocity finalVelocity, LinearVelocity currentVelocity, Time dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertVelocityTo(drivetrain.getVelocityChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(currentVelocity), dt));
    }

    public Distance getDistanceChange(LinearVelocity finalVelocity, LinearVelocity velocity, Time dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertPositionTo(drivetrain.getAngleChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(velocity), dt));
    }
}
