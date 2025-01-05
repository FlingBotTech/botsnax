package botsnax.util;

import edu.wpi.first.units.*;

public class MomentOfInertia {
    @SuppressWarnings("unchecked")
    public static Measure<Mult<Mass, Mult<Distance, Distance>>> ofUniformCylinder(Measure<Mass> mass, Measure<Distance> radius) {
        return ((Measure<Mult<Mass, Mult<Distance, Distance>>>) mass.times(radius).times(radius)).times(0.5);
    }
}
