package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import java.util.function.Function;

import static edu.wpi.first.units.Units.*;

public class BeelineSpeeds {
    private static final ChassisSpeeds STOP = new ChassisSpeeds(0, 0, 0);

    public static Function<Pose2d, ChassisSpeeds> of(Translation2d destination, LinearVelocity maxSpeed, Time stopTime, Distance threshold) {
        final double maxSpeedMPS = maxSpeed.in(MetersPerSecond);
        final double thresholdM = threshold.in(Meters);
        final double stopTimeS = stopTime.in(Seconds);

        return pose -> {
            Translation2d delta = destination.minus(pose.getTranslation());
            double distanceM = delta.getNorm();
            double stopSpeedMPS = distanceM / stopTimeS;
            double speedMPS = Math.min(maxSpeedMPS, stopSpeedMPS);

            if (distanceM > thresholdM) {
                double vx = delta.getX() / distanceM * speedMPS;
                double vy = delta.getY() / distanceM * speedMPS;

                return new ChassisSpeeds(vx, vy, 0);
            } else {
                return STOP;
            }
        };
    }
}