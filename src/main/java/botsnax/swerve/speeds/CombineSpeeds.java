package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;

public class CombineSpeeds {
    public static Function<Pose2d, ChassisSpeeds> plus(Function<Pose2d, ChassisSpeeds> f1, Function<Pose2d, ChassisSpeeds> f2) {
        return pose -> f1.apply(pose).plus(f2.apply(pose));
    }
}
