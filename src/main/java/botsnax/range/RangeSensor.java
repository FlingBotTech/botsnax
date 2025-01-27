package botsnax.range;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

import java.util.Optional;

public interface RangeSensor {
    Optional<RangeMeasurement> getDistance();
}
