package botsnax.range;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

import java.util.Optional;

import static edu.wpi.first.units.Units.Seconds;

public class CANrangeRangeSensor implements RangeSensor {
    private final CANrange canrange;

    public CANrangeRangeSensor(CANrange canrange) {
        this.canrange = canrange;
    }

    @Override
    public Optional<RangeMeasurement> getDistance() {
        StatusSignal<Distance> distanceSignal = canrange.getDistance();
        StatusSignal<Distance> stdDevSignal = canrange.getDistanceStdDev();

        return canrange.getIsDetected().getValue() ?
                Optional.of(new RangeMeasurement(
                        distanceSignal.getValue(),
                        stdDevSignal.getValue(),
                        Seconds.of(distanceSignal.getTimestamp().getTime()))) :
                Optional.empty();
    }
}
