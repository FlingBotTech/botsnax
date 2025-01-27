package botsnax.swerve.speeds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;

public class CircleAndEndpoint {
    private final Translation2d center;
    private final Distance radius;
    private final Translation2d endpoint;
    private final double endpointToTangentDistanceM;
    private final Function<Pose2d, Boolean> excludeCondition;
    private final Function<Pose2d, ChassisSpeeds> insideSpeeds;

    private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds(0, 0, 0);

    public CircleAndEndpoint(Translation2d center, Distance radius, Function<Pose2d, Boolean> excludeCondition, Function<Pose2d, ChassisSpeeds> insideSpeeds, Translation2d endpoint) {
        this.center = center;
        this.radius = radius;
        this.excludeCondition = excludeCondition;
        this.insideSpeeds = insideSpeeds;
        this.endpoint = endpoint;

        boolean endpointOutside = center.getDistance(endpoint) > radius.in(Meters);

        this.endpointToTangentDistanceM = endpointOutside ? distanceToTangent(endpoint) : 0;
    }

    public CircleAndEndpoint(Translation2d center, Distance radius, Translation2d endpoint) {
        this(
                center,
                radius,
                x -> false,
                x -> ZERO_SPEEDS,
                endpoint);
    }

    private Optional<Double> lineFromHereIntersectsCircle(Translation2d here) {
        Translation2d hereToEndpoint = endpoint.minus(here);
        double hereToEndpointSquareM = squareNorm(hereToEndpoint);

        if (hereToEndpointSquareM > 1e-6) {
            double t = -dotProduct(here.minus(center), hereToEndpoint) / hereToEndpointSquareM;
            double distance = center.getDistance(here.plus(hereToEndpoint.times(t)));

            return distance < radius.in(Meters) ? Optional.of(t) : Optional.empty();
        } else {
            return Optional.empty();
        }
    }

    private Optional<Pair<Double, ChassisSpeeds>> detect(Pose2d pose, ChassisSpeeds chassisSpeeds) {
        Translation2d here = pose.getTranslation();
        Optional<Double> depthIfAny = excludeCondition.apply(pose) ? Optional.empty() : lineFromHereIntersectsCircle(here);

        return depthIfAny.flatMap(depth -> {
            Translation2d givenVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
            double linearSpeedMPS = givenVelocity.getNorm();
            Translation2d hereToCenter = center.minus(here);
            double hereToEndpointM = here.getDistance(endpoint);
            double hereToCenterM = hereToCenter.getNorm();

            if (hereToEndpointM < endpointToTangentDistanceM) {
                return Optional.empty();
            } else if (hereToCenterM > radius.in(Meters)) {
                Rotation2d rotation = new Rotation2d(distanceToTangent(here), radius.in(Meters));
                Translation2d hereToTangent1 = hereToCenter.rotateBy(rotation);
                double distance1 = hereToTangent1.plus(here).getDistance(endpoint);
                Translation2d hereToTangent2 = hereToCenter.rotateBy(rotation.unaryMinus());
                double distance2 = hereToTangent2.plus(here).getDistance(endpoint);
                Translation2d hereToTangent = (distance1 < distance2) ? hereToTangent1 : hereToTangent2;
                double hereToTangentM = hereToTangent.getNorm();
                Translation2d velocity = hereToTangent.times(linearSpeedMPS / hereToTangentM);

                return Optional.of(new Pair<>(depth, new ChassisSpeeds(velocity.getX(), velocity.getY(), 0)));
            } else {
                Translation2d hereToEndpoint = endpoint.minus(here);
                Translation2d hereToEndpointDirection = hereToEndpoint.div(hereToEndpoint.getNorm());
                Translation2d rotated = hereToCenter.rotateBy(Rotation2d.kCW_90deg).div(hereToCenterM);
                Translation2d tangential = dotProduct(rotated, hereToEndpointDirection) > 0 ? rotated : rotated.times(-1);
                Translation2d velocity = tangential.times(linearSpeedMPS);
                ChassisSpeeds speeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), 0)
                        .plus(insideSpeeds.apply(pose));

                return Optional.of(new Pair<>(depth, speeds));
            }
        });
    }

    private double distanceToTangent(Translation2d point) {
        return Math.sqrt(squareNorm(center.minus(point)) - Math.pow(radius.in(Meters), 2));
    }

    private static double dotProduct(Translation2d t1, Translation2d t2) {
        return t1.getX() * t2.getX() + t1.getY() * t2.getY();
    }

    private static double squareNorm(Translation2d point) {
        return dotProduct(point, point);
    }

    public static Function<Pose2d, ChassisSpeeds> avoid(List<CircleAndEndpoint> circles, Function<Pose2d, ChassisSpeeds> speeds) {
        return pose -> {
            ChassisSpeeds chassisSpeeds = speeds.apply(pose);

            Optional<Pair<Double, ChassisSpeeds>> nearest = circles
                    .stream()
                    .flatMap(circle -> circle.detect(pose, chassisSpeeds).stream())
                    .min(Comparator.comparingDouble(Pair::getFirst));

            return nearest.map(Pair::getSecond).orElse(chassisSpeeds);
        };
    }
}
