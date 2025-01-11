package botsnax.swerve.sim;

import botsnax.util.MomentsOfInertia;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

import static botsnax.util.DCMotorUtil.withMotorCount;
import static edu.wpi.first.units.Units.*;

public class IdealizedSwerveDrivetrain {
    private final LinearWheelDrivetrain linearDrive;
    private final AngularWheelDrivetrain angularDrive;

    public IdealizedSwerveDrivetrain(
            Wheels wheels,
            Mass carriageMass,
            MomentOfInertia carriageMoment,
            Distance wheelToCenterDistance,
            double frictionCoefficient) {

        double radiusRatio = wheels.radius().baseUnitMagnitude() / wheelToCenterDistance.baseUnitMagnitude();
        linearDrive = new LinearWheelDrivetrain(wheels, carriageMass, frictionCoefficient);
        angularDrive = new AngularWheelDrivetrain(wheels, carriageMass, carriageMoment, frictionCoefficient, radiusRatio);
    }

    public LinearVelocity getMaxLinearSpeed() {
        return linearDrive.getMaxSpeed();
    }

    public IdealizedSwerveSim createSim(Pose2d initialPose) {
        return new IdealizedSwerveSim(this, initialPose);
    }

    public LinearWheelDrivetrain getLinearDrive() {
        return linearDrive;
    }

    public AngularWheelDrivetrain getAngularDrive() {
        return angularDrive;
    }

    public static IdealizedSwerveDrivetrain ofRectangularChassis(
            SwerveModuleConstants<?, ?, ?>[] modules,
            DCMotor motor,
            Mass carriageMass) {
        if (modules.length != 4) {
            throw new IllegalArgumentException("Number of modules must be 4");
        }

        SwerveModuleConstants<?, ?, ?> module = modules[0];
        Distance wheelToCenterDistance = Meters.of(VecBuilder.fill(module.LocationX, module.LocationY).norm());

        return new IdealizedSwerveDrivetrain(
                new Wheels(
                        4,
                        count -> withMotorCount(motor, count),
                        module.DriveMotorGearRatio,
                        Inches.of(module.WheelRadius),
                        KilogramSquareMeters.of(0),
                        Amps.of(module.SlipCurrent)
                ),
                carriageMass,
                MomentsOfInertia.ofUniformCylinder(carriageMass, wheelToCenterDistance.times(0.5)),
                wheelToCenterDistance,
                100 / carriageMass.baseUnitMagnitude());
    }
}
