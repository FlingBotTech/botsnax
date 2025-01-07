package botsnax.util;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.pow;

public class MomentsOfInertia {
    public static MomentOfInertia ofUniformCylinder(Mass mass, Distance radius) {
        return KilogramSquareMeters.of(mass.in(Kilograms) * pow(radius.in(Meters), 2)).times(0.5);
    }
}
