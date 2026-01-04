package botsnax.util;

import edu.wpi.first.units.measure.Time;

public class ValueAtTime<ValueT> {
    private final ValueT value;
    private final Time time;

    public ValueAtTime(ValueT value, Time time) {
        this.value = value;
        this.time = time;
    }
}
