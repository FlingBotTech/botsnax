package botsnax.swerve.speeds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import java.util.function.Function;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;
import static java.lang.Math.*;

public class FacingSpeeds {
    public static Function<Pose2d, ChassisSpeeds> of(Rotation2d angle, AngularVelocity maxVelocity, Time stopTime) {
        return pose -> {
            double delta = angleModulus(angle.getRadians() - pose.getRotation().getRadians());
            double reachVelocity = delta / stopTime.in(Seconds);
            double velocity = min(maxVelocity.in(RadiansPerSecond), abs(reachVelocity)) * signum(reachVelocity);

            return new ChassisSpeeds(0, 0, velocity);
        };
    };
}
