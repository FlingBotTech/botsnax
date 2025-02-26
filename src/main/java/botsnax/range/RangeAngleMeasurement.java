package botsnax.range;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public record RangeAngleMeasurement(Distance distance, Rotation2d angle, Distance stdDev, Time timestamp) {
}
