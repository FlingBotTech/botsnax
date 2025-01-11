package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class BeelineSpeeds {
    private static final ChassisSpeeds STOP = new ChassisSpeeds(0, 0, 0);

    public static Function<Pose2d, ChassisSpeeds> of(Translation2d destination, LinearVelocity maxSpeed, Distance threshold) {
        final double maxSpeedMPS = maxSpeed.in(MetersPerSecond);
        final double thresholdM = threshold.in(Meters);

        return pose -> {
            Translation2d delta = destination.minus(pose.getTranslation());
            double distance = delta.getNorm();

            if (distance > thresholdM) {
                double vx = delta.getX() / distance * maxSpeedMPS;
                double vy = delta.getY() / distance * maxSpeedMPS;

                return new ChassisSpeeds(vx, vy, 0);
            } else {
                return STOP;
            }
        };
    }
}