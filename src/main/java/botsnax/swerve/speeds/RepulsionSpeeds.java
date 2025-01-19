package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class RepulsionSpeeds {
    public static Function<Pose2d, ChassisSpeeds> ofCircle(LinearVelocity maxVelocity, Translation2d center, Distance radius, Distance margin) {
        return pose -> {
            Translation2d delta = pose.getTranslation().minus(center);
            double distance = delta.getNorm();

            if (distance < radius.plus(margin).in(Meters)) {
                double velocityScale = 1 - Math.max(0, (distance - radius.in(Meters)) / margin.in(Meters));
                double scale = Math.pow(velocityScale, 0.3) * maxVelocity.in(MetersPerSecond) / distance;
                return new ChassisSpeeds(delta.getX() * scale, delta.getY() * scale, 0);
            } else {
                return new ChassisSpeeds(0, 0, 0);
            }
        };
    }
}
