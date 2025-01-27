package botsnax.range;

import botsnax.vision.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

public class MountedRangeSensor {
    private final RangeSensor rangeSensor;
    private final Transform2d robotToSensor;

    public MountedRangeSensor(RangeSensor rangeSensor, Transform2d robotToSensor) {
        this.rangeSensor = rangeSensor;
        this.robotToSensor = robotToSensor;
    }

    public RangeSensor getSensor() {
        return rangeSensor;
    }

    public Optional<PositionEstimate> getRobotRelativeMeasurementIfAny(Pose2d robotPose) {
        return rangeSensor.getDistance().map(measurement -> {
            Pose2d sensorPose = robotPose.transformBy(robotToSensor);
            Translation2d fieldPoint = new Translation2d(measurement.distance().in(Meters), 0).rotateBy(robotToSensor.getRotation()).plus(sensorPose.getTranslation());

            return new PositionEstimate(fieldPoint, measurement.stdDev(), measurement.timestamp());
        });
    }

    public Optional<PoseEstimate> snapTo(Pose2d robotPose, Pose2d surface) {
        return getRobotRelativeMeasurementIfAny(robotPose).map(measurement -> {
            Translation2d surfaceNormal = new Translation2d(1, 0).rotateBy(surface.getRotation());
            double sensedPointDistance = dotProduct(measurement.position().minus(surface.getTranslation()), surfaceNormal);
            Translation2d displacement = surfaceNormal.times(-sensedPointDistance);
            Pose2d updatedPose = robotPose.transformBy(new Transform2d(displacement, Rotation2d.kZero));

            return new PoseEstimate(
                    updatedPose,
                    new Matrix<>(N3.instance, N1.instance, new double[]{
                            measurement.stdDev().in(Meters),
                            measurement.stdDev().in(Meters),
                            1e5
                    }),
                    measurement.timestamp());
        });
    }

    private static double dotProduct(Translation2d t1, Translation2d t2) {
        return t1.getX() * t2.getX() + t1.getY() * t2.getY();
    }
}
