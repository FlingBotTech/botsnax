package botsnax.control.motor.state;

import botsnax.control.periodic.ScalarState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface AngleState extends ScalarState<Angle> {
    AngularVelocity getVelocity();
}
