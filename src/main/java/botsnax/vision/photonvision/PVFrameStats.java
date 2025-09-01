package botsnax.vision.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import static edu.wpi.first.units.Units.Degrees;
import static java.lang.Double.NaN;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class PVFrameStats {
    private record Stats(DoubleSummaryStatistics summary, double stdDev) {}

    public static void printAllResults(PVCameraState camera, AprilTagFieldLayout layout, List<PhotonPipelineResult> results) {
        Set<Integer> tagIds = results.stream()
                .flatMap(result -> result.getTargets().stream()
                        .map(target -> target.fiducialId))
                .collect(Collectors.toSet());
        tagIds.forEach(tagId -> printTagResult(camera, layout, results, tagId));

        printMultiTagResult(camera, results);
    }

    public static void printMultiTagResult(PVCameraState camera, List<PhotonPipelineResult> results) {
        List<Optional<Pose3d>> poses = results.stream()
                .map(result -> result.getMultiTagResult()
                        .map(multi -> camera.getMultiTagPose(multi.estimatedPose.best)))
                .toList();

        System.out.println("Results for multi-tag");
        printPose2dStats(poses);
        System.out.println();
    }

    public static void printTagResult(PVCameraState camera, AprilTagFieldLayout layout, List<PhotonPipelineResult> results, int tagId) {
        final Pose3d targetPosition = layout.getTagPose(tagId).get();

        List<Optional<PhotonTrackedTarget>> targets = results.stream()
                .map(result -> result.getTargets().stream()
                        .filter(target -> target.fiducialId == tagId)
                        .findFirst())
                .toList();

        List<Optional<Pose3d>> poses = targets.stream()
                .map(targetIfAny -> targetIfAny.map(target ->
                        targetPosition
                                .transformBy(target.getBestCameraToTarget().inverse())
                                .transformBy(camera.getCameraToRobot())))
                .toList();

        System.out.println("Results for tag " + tagId);
        printPose2dStats(poses);
        printStats("rx", getTagValues(targets, target -> target.bestCameraToTarget.getRotation().getMeasureX().in(Degrees)));
        printStats("ry", getTagValues(targets, target -> target.bestCameraToTarget.getRotation().getMeasureY().in(Degrees)));
        printStats("rz", getTagValues(targets, target -> target.bestCameraToTarget.getRotation().getMeasureZ().in(Degrees)));
        printStats("d", getTagValues(targets, target -> target.bestCameraToTarget.getTranslation().getNorm()));
        System.out.println();
    }

    private static List<Double> getTagValues(List<Optional<PhotonTrackedTarget>> targets, Function<PhotonTrackedTarget, Double> f) {
        return targets.stream()
                .map(targetIfAny -> targetIfAny.map(f).orElse(NaN))
                .toList();
    }

    private static Stats getStats(List<Double> values) {
        DoubleSummaryStatistics summary = values.stream().mapToDouble(x -> x).summaryStatistics();
        DoubleSummaryStatistics stdDevSummary = values.stream().mapToDouble(x -> pow(x - summary.getAverage(), 2)).summaryStatistics();
        double stdDev = sqrt(stdDevSummary.getAverage());

        return new Stats(summary, stdDev);
    }

    private static void printPose2dStats(List<Optional<Pose3d>> poses) {
        printStats("x", poses.stream().map(poseIfAny -> poseIfAny.map(pose -> pose.toPose2d().getX()).orElse(NaN)).toList());
        printStats("y", poses.stream().map(poseIfAny -> poseIfAny.map(pose -> pose.toPose2d().getY()).orElse(NaN)).toList());
        printStats("r", poses.stream().map(poseIfAny -> poseIfAny.map(pose -> pose.toPose2d().getRotation().getDegrees()).orElse(NaN)).toList());
    }

    private static void printStats(String name, List<Double> values) {
        printStats(name, getStats(values));
    }

    private static void printStats(String name, Stats stats) {
        System.out.println(name + " -> Average: " + stats.summary.getAverage() + ", StdDev: " + stats.stdDev);
    }
}
