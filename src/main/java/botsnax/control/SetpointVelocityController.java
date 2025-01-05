package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.system.motor.MotorState;

import java.util.function.Function;

public interface SetpointVelocityController {
    Measure<Velocity<Angle>> calculate(Measure<Angle> setpoint, MotorState state);

    static MotorVelocityController ofProfile(Function<MotorState, Measure<Angle>> profile, SetpointVelocityController setpointController) {
        return state -> setpointController.calculate(profile.apply(state), state);
    }
}
