package botsnax.vision.photonvision;

import botsnax.vision.PoseEstimate;

import java.util.List;
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
}
