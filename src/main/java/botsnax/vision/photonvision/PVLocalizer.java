package botsnax.vision.photonvision;

import botsnax.vision.PoseEstimate;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;

public class PVLocalizer {
    private final List<PVCameraState> cameras;
    private final Function<List<PVCameraState>, List<PoseEstimate>> poseEstimateFunction;
    private final Consumer<PoseEstimate> poseEstimateConsumer;

    public PVLocalizer(List<PVCameraState> cameras, Function<List<PVCameraState>, List<PoseEstimate>> poseEstimateFunction, Consumer<PoseEstimate> poseEstimateConsumer) {
        this.cameras = cameras;
        this.poseEstimateFunction = poseEstimateFunction;
        this.poseEstimateConsumer = poseEstimateConsumer;

        PVCameraState.addUpdateListener(this :: update);
    }

    public List<PVCameraState> getCameras() {
        return cameras;
    }

    public void update() {
        List<PVCameraState> camerasWithUpdates = cameras.stream()
                .filter(camera -> camera.getLatestResult().isPresent())
                .toList();

        poseEstimateFunction.apply(camerasWithUpdates).forEach(poseEstimateConsumer);
    }

    public Optional<VisionSystemSim> simulate(String simulationName, AprilTagFields field, Function<PVCameraState, PhotonCameraSim> cameraSimFunction) {
        if (RobotBase.isSimulation()) {
            VisionSystemSim sim = new VisionSystemSim(simulationName);
            sim.addAprilTags(AprilTagFieldLayout.loadField(field));

            cameras.forEach(camera -> {
                sim.addCamera(cameraSimFunction.apply(camera), camera.getRobotToCamera());
            });

            return Optional.of(sim);
        } else {
            return Optional.empty();
        }
    }

    public Optional<VisionSystemSim> simulate() {
        return simulate("main", AprilTagFields.kDefaultField, camera -> {
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera.getCamera(), new SimCameraProperties());

            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
            cameraSim.enableDrawWireframe(true);

            return cameraSim;
        });
    }
}
