package botsnax.swerve.sim;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public record AngularWheelDrivetrain(Wheels wheels, Mass carriageMass, MomentOfInertia carriageMoment, double frictionCoefficient, double ratio) {
    private AngularVelocity convertVelocityFrom(AngularVelocity velocity) {
        return velocity.div(ratio);
    }

    private AngularVelocity convertVelocityTo(AngularVelocity velocity) {
        return velocity.times(ratio);
    }

    private MomentOfInertia getMoment() {
        return carriageMoment.times(ratio);
    }

    private Angle convertPositionTo(Angle position) {
        return position.times(ratio);
    }

    public AngularVelocity getVelocityChange(AngularVelocity finalVelocity, AngularVelocity currentVelocity, Time dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertVelocityTo(drivetrain.getVelocityChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(currentVelocity), dt));
    }

    public Angle getAngleChange(AngularVelocity finalVelocity, AngularVelocity velocity, Time dt) {
        WheelDrivetrain drivetrain = new WheelDrivetrain(wheels, carriageMass, getMoment(), frictionCoefficient);

        return convertPositionTo(drivetrain.getAngleChange(convertVelocityFrom(finalVelocity), convertVelocityFrom(velocity), dt));
    }
}
