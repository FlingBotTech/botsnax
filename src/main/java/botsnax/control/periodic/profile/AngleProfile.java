package botsnax.control.periodic.profile;

import botsnax.control.periodic.ScalarState;
import edu.wpi.first.units.measure.Angle;

public class AngleProfile {
    public static <StateT extends ScalarState<Angle>> Profile<Angle, StateT> withBounds(Angle bound1, Angle bound2, Profile<Angle, StateT> profile) {
        Angle lowerBound = bound1.lt(bound2) ? bound1 : bound2;
        Angle upperBound = bound1.gt(bound2) ? bound1 : bound2;

        return (state, priorValue) -> {
            Angle angle = profile.getValue(state, priorValue);

            if (angle.lt(lowerBound)) {
                return lowerBound;
            } else if (angle.gt(upperBound)) {
                return upperBound;
            } else {
                return angle;
            }
        };
    }
}
