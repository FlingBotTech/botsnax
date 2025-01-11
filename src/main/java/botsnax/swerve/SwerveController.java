package botsnax.swerve;

import botsnax.swerve.factory.GenericSwerveDrivetrainFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;

public interface SwerveController {
    GenericSwerveDrivetrain getDrivetrain();
    Pose2d getPose();
    void setPose(Pose2d pose);
    void setFieldSpeeds(Function<Pose2d, ChassisSpeeds> speeds);
}
