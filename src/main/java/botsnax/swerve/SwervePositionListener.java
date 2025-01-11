package botsnax.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwervePositionListener {
    Pose2d update(Rotation2d heading, SwerveModulePosition[] positions);
}
