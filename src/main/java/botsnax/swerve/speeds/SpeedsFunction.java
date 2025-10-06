package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;

public interface SpeedsFunction extends Function<Pose2d, ChassisSpeeds> {
    ChassisSpeeds apply(Pose2d pose);
}
