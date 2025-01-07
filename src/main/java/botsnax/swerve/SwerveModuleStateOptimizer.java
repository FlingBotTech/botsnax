package botsnax.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Radians;

public class SwerveModuleStateOptimizer extends SwerveModuleState {
    public SwerveModuleState apply(SwerveModuleState state, Rotation2d currentAngle) {
        speedMetersPerSecond = state.speedMetersPerSecond;
        angle = state.angle;

        optimize(currentAngle);
        cosineScale(currentAngle);

        return this;
    }
}
