package botsnax.range;

import botsnax.vision.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class RangeSensorPair {
    private final DoublePublisher distancePublisher;
    private final DoublePublisher anglePublisher;

    private final MountedRangeSensor leftRangeSensor;
    private final MountedRangeSensor rightRangeSensor;

    public RangeSensorPair(String name, MountedRangeSensor leftRangeSensor, MountedRangeSensor rightRangeSensor) {
        this.leftRangeSensor = leftRangeSensor;
        this.rightRangeSensor = rightRangeSensor;

        this.distancePublisher = NetworkTableInstance.getDefault().getDoubleTopic(name + " Pair Distance").publish();
        this.anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic(name + " Pair Angle").publish();
    }

    public MountedRangeSensor getLeft() {
        return leftRangeSensor;
    }

    public MountedRangeSensor getRight() {
        return rightRangeSensor;
    }

    public Optional<PoseEstimate> getPoseEstimate(Pose2d robotPose, Pose2d plane) {
        return getDistanceAngleFromPlaneIfAny().map(normDistance -> {
            Distance lateralDistance = Meters.of(robotPose.getTranslation().minus(plane.getTranslation()).rotateBy(plane.getRotation().unaryMinus()).getY());
            Pose2d fieldPose = getFieldPose(normDistance.distance(), lateralDistance, normDistance.angle(), plane);

            distancePublisher.set(normDistance.distance().in(Meters));

            return new PoseEstimate(
                    fieldPose,
                    new Matrix<>(N3.instance, N1.instance, new double[]{
                            normDistance.stdDev().in(Meters),
                            normDistance.stdDev().in(Meters),
                            1e-10,
                    }),
                    normDistance.timestamp()
            );
        });
    }

    public Optional<PoseEstimate> getPoseEstimate(RangeMeasurement lateralDistance, Pose2d plane) {
        return getDistanceAngleFromPlaneIfAny().map(normDistance -> {
            Pose2d fieldPose = getFieldPose(normDistance.distance(), lateralDistance.distance(), normDistance.angle(), plane);
            Distance stdDev = normDistance.stdDev().plus(lateralDistance.stdDev());
            Angle angularStdDev = Radians.of(1e5);

            return new PoseEstimate(
                    fieldPose,
                    new Matrix<>(N3.instance, N1.instance, new double[]{
                            stdDev.in(Meters),
                            stdDev.in(Meters),
                            1e-10
                    }),
                    normDistance.timestamp().plus(lateralDistance.timestamp()).div(2)
            );
        });
    }

    private static Pose2d getFieldPose(Distance normDistance, Distance lateralDistance, Rotation2d angle, Pose2d plane) {
        Pose2d planeRelativePose = new Pose2d(
                new Translation2d(normDistance.in(Meters), lateralDistance.in(Meters)),
                angle.rotateBy(Rotation2d.k180deg));

        return new Pose2d(
                planeRelativePose.getTranslation().rotateBy(plane.getRotation()).plus(plane.getTranslation()),
                planeRelativePose.getRotation().rotateBy(plane.getRotation())
        );
    }

    private Optional<RangeAngleMeasurement> getDistanceAngleFromPlaneIfAny() {
        return getMeasurementPairIfAny().map(pair -> {
                    Translation2d delta = pair.right().position().minus(pair.left().position());
                    Rotation2d rotation = new Rotation2d(delta.getX(), delta.getY())
                            .rotateBy(Rotation2d.kCCW_90deg)
                            .rotateBy(leftRangeSensor.getRobotToSensor().getRotation())
                            .unaryMinus();

                    anglePublisher.set(rotation.getDegrees());

                    return new RangeAngleMeasurement(
                            getMinDistanceToOrigin(pair.left().position(), pair.right().position()),
                            rotation,
                            pair.left().stdDev().plus(pair.right().stdDev()),
                            pair.left().timestamp().plus(pair.right().timestamp()).div(2)
                    );
                }
        );
    }

    private static Distance getMinDistanceToOrigin(Translation2d point1, Translation2d point2) {
        Translation2d D = point2.minus(point1);
        double t = -dotProduct(point1, D) / dotProduct(D, D);

        return Meters.of(point1.plus(D.times(t)).getNorm());
    }

    private static double dotProduct(Translation2d t1, Translation2d t2) {
        return t1.getX() * t2.getX() + t1.getY() * t2.getY();
    }

    private Optional<PositionEstimatePair> getMeasurementPairIfAny() {
        return leftRangeSensor.getFieldRelativeMeasurementIfAny().flatMap(left ->
                rightRangeSensor.getFieldRelativeMeasurementIfAny().map(right ->
                        new PositionEstimatePair(left, right)));
    }
}
