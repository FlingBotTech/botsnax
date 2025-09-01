package botsnax.vision.photonvision.command;

import botsnax.vision.photonvision.PVCameraState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class PVProcessFramesCommand extends SequentialCommandGroup {
    private final List<PhotonPipelineResult> frames;

    public PVProcessFramesCommand(PVCameraState camera, int frameCount, Consumer<List<PhotonPipelineResult>> consumer) {
        this.frames = new ArrayList<>(frameCount);

        addCommands(
                new PVCollectFramesCommand(camera, frameCount),
                new InstantCommand(() -> consumer.accept(frames))
        );
    }

    private class PVCollectFramesCommand extends Command {
        private final PVCameraState camera;
        private final int frameCount;

        private PVCollectFramesCommand(PVCameraState camera, int frameCount) {
            this.camera = camera;
            this.frameCount = frameCount;
        }

        private boolean frameCountReached() {
            return frames.size() >= frameCount;
        }

        @Override
        public void execute() {
            camera.getResults().forEach(result -> {
                if (!frameCountReached()) {
                    System.out.println(frames.size() + " frames reached");
                    frames.add(result);
                }
            });
        }

        @Override
        public void initialize() {
            frames.clear();
        }

        @Override
        public boolean isFinished() {
            return frameCountReached();
        }
    }
}
