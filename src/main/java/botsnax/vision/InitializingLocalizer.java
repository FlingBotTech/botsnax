package botsnax.vision;

import botsnax.swerve.SwerveController;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.Consumer;

public class InitializingLocalizer implements Consumer<PoseEstimate> {
    private boolean initialized = false;
    private final Consumer<Pose2d> poseConsumer;
    private final Consumer<PoseEstimate> poseEstimateConsumer;

    public InitializingLocalizer(SwerveController swerveController) {
        this(swerveController :: setPose, swerveController :: addVisionMeasurement);
    }

    public InitializingLocalizer(Consumer<Pose2d> poseConsumer, Consumer<PoseEstimate> poseEstimateConsumer) {
        this.poseConsumer = poseConsumer;
        this.poseEstimateConsumer = poseEstimateConsumer;
    }


    @Override
    public void accept(PoseEstimate poseEstimate) {
        if (!initialized) {
            poseConsumer.accept(poseEstimate.pose());
            initialized = true;
        } else {
            poseEstimateConsumer.accept(poseEstimate);
        }
    }
}
