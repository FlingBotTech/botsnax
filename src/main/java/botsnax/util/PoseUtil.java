package botsnax.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

public class PoseUtil {
    public static Pose2d offsetOf(Pose2d pose, Distance distance) {
        Translation2d displacementVector = new Translation2d(1, 0)
                .rotateBy(pose.getRotation())
                .times(-1)
                .times(distance.in(Meters));

        return new Pose2d(pose.getTranslation().plus(displacementVector), pose.getRotation());
    }

    public static Pose2d flipRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().rotateBy(Rotation2d.k180deg));
    }
}
