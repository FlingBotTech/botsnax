package botsnax.swerve.sim;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.Radians;

public record LinearWheelDrivetrain(Wheels wheels, Measure<Mass> carriageMass, double frictionCoefficient) {
    private Measure<Distance> convertPositionTo(Measure<Angle> angle) {
        return wheels.radius().times(angle.in(Radians));
    }

    public Measure<Velocity<Angle>> convertVelocityFrom(Measure<Velocity<Distance>> velocity) {
        return BaseUnits.Angle.per(BaseUnits.Time).of(velocity.baseUnitMagnitude() / wheels.radius().baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    private Measure<Velocity<Distance>> convertVelocityTo(Measure<Velocity<Angle>> velocity) {
        return (Measure<Velocity<Distance>>) velocity.times(wheels.radius());
    }

    @SuppressWarnings("unchecked")
    private Measure<Mult<Mass, Mult<Distance, Distance>>> getMoment() {
        return (Measure<Mult<Mass, Mult<Distance, Distance>>>) carriageMass.times(wheels.radius().times(wheels.radius()));
    }

    public Measure<Velocity<Distance>> getMaxSpeed() {
        return convertVelocityTo(wheels.getMaxSpeed());
    }

    public Measure<Velocity<Distance>> getVelocityChange(Measure<Velocity<Distance>> finalVelocity, Measure<Velocity<Distance>> currentVelocity, Measure<Time> dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertVelocityTo(drivetrain.getVelocityChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(currentVelocity), dt));
    }

    public Measure<Distance> getDistanceChange(Measure<Velocity<Distance>> finalVelocity, Measure<Velocity<Distance>> velocity, Measure<Time> dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertPositionTo(drivetrain.getAngleChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(velocity), dt));
    }
}
