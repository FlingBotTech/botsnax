package botsnax.range;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

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

    public Transform2d getRobotToSensor() {
        return robotToSensor;
    }

    public Optional<PositionEstimate> getFieldRelativeMeasurementIfAny() {
        return rangeSensor.getDistance().map(measurement -> {
            Translation2d fieldPoint = new Translation2d(measurement.distance().in(Meters), 0)
                    .rotateBy(robotToSensor.getRotation())
                    .plus(robotToSensor.getTranslation())
                    ;

            return new PositionEstimate(fieldPoint, measurement.stdDev(), measurement.timestamp());
        });
    }
}
