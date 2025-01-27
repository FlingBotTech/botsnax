package botsnax.range;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public record RangeMeasurement(Distance distance, Distance stdDev, Time timestamp) {}
