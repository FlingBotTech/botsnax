package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.system.motor.MotorState;

public interface MotorVelocityController {
    Measure<Velocity<Angle>> calculate(MotorState state);
}
