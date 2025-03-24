package botsnax.range;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;
import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;

public class MountedRangeSensor {
    private final RangeSensor rangeSensor;
    private final Transform2d robotToSensor;
    private final Function<Double, Double> filter;

    public MountedRangeSensor(RangeSensor rangeSensor, Transform2d robotToSensor) {
        this(rangeSensor, robotToSensor, Function.identity());
    }

    public MountedRangeSensor(RangeSensor rangeSensor, Transform2d robotToSensor, Function<Double, Double> filter) {
        this.rangeSensor = rangeSensor;
        this.robotToSensor = robotToSensor;
        this.filter = filter;
    }

    public RangeSensor getSensor() {
        return rangeSensor;
    }

    public Transform2d getRobotToSensor() {
        return robotToSensor;
    }

    public Optional<PositionEstimate> getFieldRelativeMeasurementIfAny() {
        return rangeSensor.getDistance().map(measurement -> {
            double distance = filter.apply(measurement.distance().in(Meters));
            Translation2d fieldPoint = new Translation2d(distance, 0)
                    .rotateBy(robotToSensor.getRotation())
                    .plus(robotToSensor.getTranslation())
                    ;

            return new PositionEstimate(fieldPoint, measurement.stdDev(), measurement.timestamp());
        });
    }
}
