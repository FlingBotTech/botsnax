package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SpeedsFunction {
    ChassisSpeeds apply(Pose2d pose);
}
