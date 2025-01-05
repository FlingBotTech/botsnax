package botsnax.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Radians;

public class SwerveModuleStateOptimizer extends SwerveModuleState {
    public SwerveModuleState apply(SwerveModuleState state, Rotation2d currentAngle) {
        SwerveModuleState temp = optimize(state, currentAngle);

        speedMetersPerSecond = temp.speedMetersPerSecond;
        angle = temp.angle;

        adjustSpeedsForSteeringError(Radians.of(currentAngle.getRadians()));

        return this;
    }

    private void adjustSpeedsForSteeringError(Measure<Angle> currentAngle) {
        Measure<Angle> angleToSet = Radians.of(angle.getRadians());
        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        Measure<Angle> steerMotorError = angleToSet.minus(currentAngle);
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosineScalar = Math.cos(steerMotorError.in(Radians));
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }

        speedMetersPerSecond *= cosineScalar;
    }
}
