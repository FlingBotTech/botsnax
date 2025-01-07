package botsnax.swerve.phoenix;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public record VelocityConversion(SwerveModuleConstants<?, ?, ?> module, AngularVelocity angularVelocity) {
    public LinearVelocity toLinearVelocity() {
        return Inches.of(module.WheelRadius)
                .times(angularVelocity.in(RadiansPerSecond))
                .div(module.DriveMotorGearRatio)
                .per(Second);
    }
}
