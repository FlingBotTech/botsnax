package botsnax.range;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public record PositionEstimate(Translation2d position, Distance stdDev, Time timestamp) {
}
