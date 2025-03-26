package botsnax.swerve;

import botsnax.vision.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;

public interface SwerveController {
    GenericSwerveDrivetrain getDrivetrain();
    Pose2d getPose();
    Pose2d getSimPose();
    void setPose(Pose2d pose);
    void setSimPose(Pose2d pose);
    void setRobotSpeeds(Function<Pose2d, ChassisSpeeds> speeds);
    void setFieldSpeeds(Function<Pose2d, ChassisSpeeds> speeds);
    void addVisionMeasurement(PoseEstimate estimate);
}
