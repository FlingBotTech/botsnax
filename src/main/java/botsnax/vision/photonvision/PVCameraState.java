package botsnax.vision.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Seconds;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.util.Comparator.comparing;

public class PVCameraState {
    private static final List<PVCameraState> ALL_CAMERAS = new ArrayList<>();
    private static final List<Runnable> updateListeners = new ArrayList<>();

    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private final Transform3d cameraToRobot;

    private List<PhotonPipelineResult> results = List.of();
    private Optional<PhotonPipelineResult> latestResult = Optional.empty();

    private final PVPipeline<?>[] pipelines;
    private int activePipelineIndex = 0;

    public PVCameraState(String cameraName, Transform3d robotToCamera, PVPipeline<?> ... pipelines) {
        this(new PhotonCamera(cameraName), robotToCamera, pipelines);
    }

    public PVCameraState(PhotonCamera camera, Transform3d robotToCamera, PVPipeline<?> ... pipelines) {
        this.camera = camera;
        this.robotToCamera = robotToCamera;
        this.cameraToRobot = robotToCamera.inverse();
        this.pipelines = pipelines;

        ALL_CAMERAS.add(this);
    }

    public List<PhotonPipelineResult> getResults() {
        return results;
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        return latestResult;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    public Transform3d getCameraToRobot() {
        return cameraToRobot;
    }

    public void setPipelineIndex(int pipelineIndex) {
        activePipelineIndex = pipelineIndex;
        camera.setPipelineIndex(pipelineIndex);
    }

    public Optional<Pose3d> getMultiTagPose() {
        return latestResult
                .flatMap(result -> result
                        .getMultiTagResult()
                        .map(multiTagResult -> getMultiTagPose(multiTagResult.estimatedPose.best)));
    }

    public Pose3d getMultiTagPose(Transform3d targetToCamera) {
        Pose3d cameraPose = new Pose3d().transformBy(targetToCamera);
        return cameraPose.transformBy(cameraToRobot);
    }

    public Time getLatestResultTime() {
        return Seconds.of(latestResult.get().getTimestampSeconds());
    }

    public List<Translation2d> getTargetCenters() {
        return latestResult.map(result -> result.getTargets().stream().map(target -> {
            double minX = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY;
            double minY = Double.POSITIVE_INFINITY;
            double maxY = Double.NEGATIVE_INFINITY;

            for (TargetCorner corner : target.getMinAreaRectCorners()) {
                minX = min(corner.x, minX);
                maxX = max(corner.x, maxX);
                minY = min(corner.y, minY);
                maxY = max(corner.y, maxY);
            }

            return new Translation2d((minX + maxX) / 2, (minY + maxY) / 2);
        }).toList()).orElse(List.of());
    }

    public void update() {
        results = camera.getAllUnreadResults();
        latestResult = results
                .stream()
                .max(comparing(PhotonPipelineResult::getTimestampSeconds));

        if (pipelines.length > 0) {
            pipelines[activePipelineIndex].update(this);
        }
    }

    public static PVCameraState getCamera(String name) {
        return ALL_CAMERAS.stream()
                .filter(camera -> camera.camera.getName().equals(name))
                .findFirst()
                .orElseThrow();
    }

    public static void addUpdateListener(Runnable updateListener) {
        updateListeners.add(updateListener);
    }

    public static void updateAllCameras() {
        ALL_CAMERAS.forEach(PVCameraState::update);
        updateListeners.forEach(Runnable::run);
    }
}
