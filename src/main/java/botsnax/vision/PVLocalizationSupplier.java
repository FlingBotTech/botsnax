package botsnax.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public class PVLocalizationSupplier implements Supplier<Optional<PoseEstimate>> {
    private final PhotonCamera camera;
    private final Transform3d cameraToRobot;

    public PVLocalizationSupplier(PhotonCamera camera, Transform3d cameraToRobot) {
        this.camera = camera;
        this.cameraToRobot = cameraToRobot;
    }

    @Override
    public Optional<PoseEstimate> get() {
        PhotonPipelineResult result = camera.getLatestResult();
        PNPResult poseResult = result.getMultiTagResult().estimatedPose;

        if (poseResult.isPresent) {
            Pose3d cameraPose = new Pose3d().transformBy(poseResult.best);
            Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

            return Optional.of(new PoseEstimate(
                    robotPose.toPose2d(),
                    new Matrix<>(N3.instance, N1.instance, new double[] {0.1, 0.1, 1e5}),
                    Seconds.of(result.getTimestampSeconds())));
        } else {
            return Optional.empty();
        }
    }
}
