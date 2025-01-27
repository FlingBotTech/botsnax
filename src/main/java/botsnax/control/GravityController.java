package botsnax.control;

import edu.wpi.first.units.measure.Voltage;

public interface GravityController extends MotorController {
    Voltage getMaxFeedForward();
}
