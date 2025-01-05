package botsnax.swerve.sim;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import botsnax.util.MomentOfInertia;

import java.util.function.Function;

import static edu.wpi.first.units.Units.*;

public class SwerveDrivetrain {
    private final LinearWheelDrivetrain linearDrive;
    private final AngularWheelDrivetrain angularDrive;

    public SwerveDrivetrain(
            Wheels wheels,
            Measure<Mass> carriageMass,
            Measure<Mult<Mass, Mult<Distance, Distance>>> carriageMoment,
            Measure<Distance> wheelToCenterDistance,
            double frictionCoefficient) {

        double radiusRatio = wheels.radius().baseUnitMagnitude() / wheelToCenterDistance.baseUnitMagnitude();
        linearDrive = new LinearWheelDrivetrain(wheels, carriageMass, frictionCoefficient);
        angularDrive = new AngularWheelDrivetrain(wheels, carriageMass, carriageMoment, frictionCoefficient, radiusRatio);
    }

    public Measure<Velocity<Distance>> getMaxLinearSpeed() {
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

    public static SwerveDrivetrain ofRectangularChassis(
            SwerveModuleConstants[] modules,
            Function<Integer,DCMotor> motorSupplier,
            Measure<Mass> carriageMass) {
        if (modules.length != 4) {
            throw new IllegalArgumentException("Number of modules must be 4");
        }

        SwerveModuleConstants module = modules[0];
        Measure<Distance> wheelToCenterDistance = Meters.of(VecBuilder.fill(module.LocationX, module.LocationY).norm());

        return new SwerveDrivetrain(
                new Wheels(
                        4,
                        motorSupplier,
                        module.DriveMotorGearRatio,
                        Inches.of(module.WheelRadius),
                        Kilograms.mult(Meters.mult(Meters)).of(0),
                        Amps.of(module.SlipCurrent)
                ),
                carriageMass,
                MomentOfInertia.ofUniformCylinder(carriageMass, wheelToCenterDistance.times(0.5)),
                wheelToCenterDistance,
                100 / carriageMass.baseUnitMagnitude());
    }
}
