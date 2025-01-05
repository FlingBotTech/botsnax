package botsnax.swerve.phoenix;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

public record VelocityConversion(SwerveModuleConstants module, Measure<Velocity<Angle>> angularVelocity) {
    public Measure<Velocity<Distance>> toLinearVelocity() {
        return MetersPerSecond.of(
                angularVelocity.in(RadiansPerSecond) *
                Inches.of(module.WheelRadius).in(Meters) /
                module.DriveMotorGearRatio);
    }
}
