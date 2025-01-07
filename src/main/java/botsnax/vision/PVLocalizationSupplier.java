package botsnax.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PnpResult;

import java.util.Comparator;
import java.util.List;
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

    private static Optional<PhotonPipelineResult> getLatest(List<PhotonPipelineResult> results) {
        return results.stream().max(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds));
    }

    @Override
    public Optional<PoseEstimate> get() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Optional<PhotonPipelineResult> resultIfAny = results
                .stream()
                .max(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds));

        return resultIfAny
                .flatMap(result -> result.getMultiTagResult().map(
                    multiTagResult -> {
                        PnpResult poseResult = multiTagResult.estimatedPose;

                        Pose3d cameraPose = new Pose3d().transformBy(poseResult.best);
                        Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

                        return new PoseEstimate(
                                robotPose.toPose2d(),
                                new Matrix<>(N3.instance, N1.instance, new double[]{0.1, 0.1, 1e5}),
                                Seconds.of(result.getTimestampSeconds()));
                    }
            )
        );
    }
}
