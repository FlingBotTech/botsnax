package botsnax.vision.photonvision;

import botsnax.vision.PoseStdDev;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static java.lang.Math.cos;
import static java.lang.Math.pow;

public interface PVNoiseModel {
    PoseStdDev getMultiTagStdDev(MultiTargetPNPResult multiResult, PhotonPipelineResult result);
    PoseStdDev getTargetStdDev(PhotonTrackedTarget target);

    static PVNoiseModel getDefault(double multiDistanceCoefficient, double multiRotationCoefficient) {
        return new PVNoiseModel() {
            @Override
            public PoseStdDev getMultiTagStdDev(MultiTargetPNPResult multiResult, PhotonPipelineResult result) {
                return getStdDev(result.getBestTarget(), multiDistanceCoefficient, multiRotationCoefficient);
            }

            @Override
            public PoseStdDev getTargetStdDev(PhotonTrackedTarget target) {
                return getStdDev(target, multiDistanceCoefficient * 5, multiRotationCoefficient * 5);
            }

            private static PoseStdDev getStdDev(PhotonTrackedTarget target, double distanceCoefficient, double rotationCoefficient) {
                double distanceSquared = squareNorm(target.bestCameraToTarget.getTranslation());
                double azimuthCorrection = 9 * pow(cos(target.bestCameraToTarget.getRotation().getZ()), 2) + 1;
                Distance distanceStdDev = Meters.of(distanceCoefficient * distanceSquared * azimuthCorrection);
                Angle rotationStdDev = Degrees.of(rotationCoefficient * distanceSquared * azimuthCorrection);

                return new PoseStdDev(distanceStdDev, distanceStdDev, rotationStdDev);
            }
        };
    }

    private static double squareNorm(Translation3d cameraPosition) {
        double x = cameraPosition.getX();
        double y = cameraPosition.getY();
        double z = cameraPosition.getZ();

        return x * x + y * y + z * z;
    }
}
