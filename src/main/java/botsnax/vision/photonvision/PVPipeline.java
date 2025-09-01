package botsnax.vision.photonvision;

import java.util.function.Consumer;
import java.util.function.Function;

public class PVPipeline<T> {
    private final Function<PVCameraState, T> processFunction;
    private final Consumer<T> consumer;


    public PVPipeline(Function<PVCameraState, T> processFunction, Consumer<T> consumer) {
        this.processFunction = processFunction;
        this.consumer = consumer;
    }

    public void update(PVCameraState camera) {
        consumer.accept(processFunction.apply(camera));
    }
}
