package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.AngularVelocity;

public interface MotorVelocityController {
    AngularVelocity calculate(MotorState state);
}
