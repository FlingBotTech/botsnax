package botsnax.swerve.sim;

import edu.wpi.first.units.*;

public record AngularWheelDrivetrain(Wheels wheels, Measure<Mass> carriageMass, Measure<Mult<Mass, Mult<Distance, Distance>>> carriageMoment, double frictionCoefficient, double ratio) {
    private Measure<Velocity<Angle>> convertVelocityFrom(Measure<Velocity<Angle>> velocity) {
        return velocity.divide(ratio);
    }

    private Measure<Velocity<Angle>> convertVelocityTo(Measure<Velocity<Angle>> velocity) {
        return velocity.times(ratio);
    }

    private Measure<Mult<Mass, Mult<Distance, Distance>>> getMoment() {
        return carriageMoment.times(ratio);
    }

    private Measure<Angle> convertPositionTo(Measure<Angle> position) {
        return position.times(ratio);
    }

    public Measure<Velocity<Angle>> getVelocityChange(Measure<Velocity<Angle>> finalVelocity, Measure<Velocity<Angle>> currentVelocity, Measure<Time> dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertVelocityTo(drivetrain.getVelocityChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(currentVelocity), dt));
    }

    public Measure<Angle> getAngleChange(Measure<Velocity<Angle>> finalVelocity, Measure<Velocity<Angle>> velocity, Measure<Time> dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertPositionTo(drivetrain.getAngleChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(velocity), dt));
    }
}
