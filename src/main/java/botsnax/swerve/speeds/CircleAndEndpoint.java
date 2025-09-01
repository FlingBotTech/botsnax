package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Double.NaN;
import static java.lang.Double.POSITIVE_INFINITY;

public class CircleAndEndpoint {
    private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds(0, 0, 0);

    public static class CircleState {
        private final CircleAndEndpoint circle;
        private String status = "none";
        private boolean detected = false;
        private double depth = NaN;
        private ChassisSpeeds chassisSpeeds = ZERO_SPEEDS;
        private ChassisSpeeds insideSpeeds = ZERO_SPEEDS;

        private CircleState(CircleAndEndpoint circle) {
            this.circle = circle;
        }

        private void reset() {
            status = "undefined";
            detected = false;
            depth = POSITIVE_INFINITY;
            chassisSpeeds = ZERO_SPEEDS;
            insideSpeeds = ZERO_SPEEDS;
        }

        public CircleAndEndpoint getCircle() {
            return circle;
        }

        public String getStatus() {
            return status;
        }

        public boolean isDetected() {
            return detected;
        }

        public double getDepth() {
            return depth;
        }

        public ChassisSpeeds getChassisSpeeds() {
            return chassisSpeeds;
        }

        public ChassisSpeeds getInsideSpeeds() {
            return insideSpeeds;
        }
    }

    public static class State {
        private CircleState nearest;
        private final CircleState[] all;

        public State(List<CircleAndEndpoint> circles) {
            all = new CircleState[circles.size()];

            for (int i = 0; i < circles.size(); i++) {
                all[i] = new CircleState(circles.get(i));
            }
        }

        private CircleState update(Pose2d pose, ChassisSpeeds speeds) {
            for (CircleState circleState : all) {
                circleState.circle.detect(pose, speeds, circleState);
            }

            nearest = Arrays.stream(all).min(Comparator.comparingDouble(CircleState:: getDepth)).orElseThrow();

            return nearest;
        }
    }

    public static class DefaultLogger implements Consumer<State> {
        private final BooleanPublisher detectedP;
        private final StringPublisher statusP;

        public DefaultLogger() {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("Avoidance");
            detectedP = table.getBooleanTopic("detected").publish();
            statusP = table.getStringTopic("status").publish();
        }

        @Override
        public void accept(State state) {
            detectedP.accept(state.nearest.isDetected());
            statusP.accept(state.nearest.getStatus());
        }
    }

    private final Translation2d center;
    private final Distance radius;
    private final Function<Pose2d, AngularVelocity> rotationFunction;
    private final Translation2d endpoint;
    private final double endpointToTangentDistanceM;
    private final Function<Pose2d, Boolean> excludeCondition;
    private final Function<Pose2d, ChassisSpeeds> insideSpeeds;

    public CircleAndEndpoint(Translation2d center, Distance radius, Function<Pose2d, AngularVelocity> rotationFunction, Function<Pose2d, Boolean> excludeCondition, Function<Pose2d, ChassisSpeeds> insideSpeeds, Translation2d endpoint) {
        this.center = center;
        this.radius = radius;
        this.rotationFunction = rotationFunction;
        this.excludeCondition = excludeCondition;
        this.insideSpeeds = insideSpeeds;
        this.endpoint = endpoint;

        boolean endpointOutside = center.getDistance(endpoint) > radius.in(Meters);

        this.endpointToTangentDistanceM = endpointOutside ? distanceToTangent(endpoint) : 0;
    }

    public CircleAndEndpoint(Translation2d center, Distance radius, Function<Pose2d, Boolean> excludeCondition, Function<Pose2d, ChassisSpeeds> insideSpeeds, Translation2d endpoint) {
        this(center, radius, pose -> RadiansPerSecond.zero(), excludeCondition, insideSpeeds, endpoint);
    }

    public CircleAndEndpoint(Translation2d center, Distance radius, Translation2d endpoint) {
        this(
                center,
                radius,
                x -> RadiansPerSecond.zero(),
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

    private void detect(Pose2d pose, ChassisSpeeds chassisSpeeds, CircleState circleState) {
        Translation2d here = pose.getTranslation();
        boolean excluded = excludeCondition.apply(pose);
        Optional<Double> depthIfAny = excluded ? Optional.empty() : lineFromHereIntersectsCircle(here);

        circleState.reset();

        if (excluded) {
            circleState.status = "excluded";
        } else if (depthIfAny.isEmpty()) {
            circleState.status = "missed";
        }

        depthIfAny.ifPresent(depth -> {
            circleState.depth = depth;

            Translation2d givenVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
            double linearSpeedMPS = givenVelocity.getNorm();
            Translation2d hereToCenter = center.minus(here);
            double hereToEndpointM = here.getDistance(endpoint);
            double hereToCenterM = hereToCenter.getNorm();

            if (hereToEndpointM < endpointToTangentDistanceM) {
                circleState.status = "leaving";
            } else if (hereToCenterM > radius.in(Meters)) {
                Rotation2d rotation = new Rotation2d(distanceToTangent(here), radius.in(Meters));
                Translation2d hereToTangent1 = hereToCenter.rotateBy(rotation);
                double distance1 = hereToTangent1.plus(here).getDistance(endpoint);
                Translation2d hereToTangent2 = hereToCenter.rotateBy(rotation.unaryMinus());
                double distance2 = hereToTangent2.plus(here).getDistance(endpoint);
                Translation2d hereToTangent = (distance1 < distance2) ? hereToTangent1 : hereToTangent2;
                double hereToTangentM = hereToTangent.getNorm();
                Translation2d velocity = hereToTangent.times(linearSpeedMPS / hereToTangentM);
                AngularVelocity rotationVelocity = rotationFunction.apply(pose);
                ChassisSpeeds speeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), rotationVelocity.in(RadiansPerSecond));

                circleState.status = "approaching";
                circleState.detected = true;
                circleState.chassisSpeeds = speeds;
            } else {
                Translation2d hereToEndpoint = endpoint.minus(here);
                Translation2d hereToEndpointDirection = hereToEndpoint.div(hereToEndpoint.getNorm());
                Translation2d rotated = hereToCenter.rotateBy(Rotation2d.kCW_90deg).div(hereToCenterM);
                Translation2d tangential = dotProduct(rotated, hereToEndpointDirection) > 0 ? rotated : rotated.times(-1);
                Translation2d velocity = tangential.times(linearSpeedMPS);
                AngularVelocity rotationVelocity = rotationFunction.apply(pose);
                ChassisSpeeds insideSpeedsValue = insideSpeeds.apply(pose);
                ChassisSpeeds speeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), rotationVelocity.in(RadiansPerSecond))
                        .plus(insideSpeedsValue);

                circleState.status = "inside";
                circleState.detected = true;
                circleState.insideSpeeds = insideSpeedsValue;
                circleState.chassisSpeeds = speeds;
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
        return avoid(circles, speeds, null);
    }

    public static Function<Pose2d, ChassisSpeeds> avoid(List<CircleAndEndpoint> circles, Function<Pose2d, ChassisSpeeds> speeds, Consumer<State> logger) {
        State state = new State(circles);

        return pose -> {
            ChassisSpeeds chassisSpeeds = speeds.apply(pose);
            CircleState nearest = state.update(pose, chassisSpeeds);

            if (logger != null) {
                logger.accept(state);
            }

            return nearest.isDetected() ? nearest.chassisSpeeds : chassisSpeeds;
        };
    }
}
