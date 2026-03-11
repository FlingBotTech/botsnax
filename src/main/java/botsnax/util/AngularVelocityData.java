package botsnax.util;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.ArrayList;
import java.util.List;

public class AngularVelocityData {
    private final List<AngularVelocity> velocities = new ArrayList<>();

    public void add(AngularVelocity velocity) {
        velocities.add(velocity);
    }

    public AngularVelocity summarize() {
        double average = velocities
                .subList(velocities.size() / 3, 2 * velocities.size() / 3)
                .stream()
                .mapToDouble(Measure::baseUnitMagnitude)
                .average()
                .orElseThrow();

        return BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(average);
    }
}
