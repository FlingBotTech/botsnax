package botsnax.swerve.listener;

import botsnax.swerve.SwerveOdometryUpdater.PoseUpdateListener;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseLogger implements PoseUpdateListener {
    private final Field2d field;

    public PoseLogger() {
        field = new Field2d();
        SmartDashboard.putData(field);
    }

    public Pose2d getPose() {
        return field.getRobotPose();
    }

    @Override
    public void onPoseUpdate(Pose2d pose) {
        field.setRobotPose(pose);
    }
}
