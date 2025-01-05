package botsnax.swerve.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveSim {
    Pose2d getPose();
    ChassisSpeeds getVelocity();
}
