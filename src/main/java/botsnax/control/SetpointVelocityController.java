package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.function.Function;

public interface SetpointVelocityController {
    AngularVelocity calculate(Angle setpoint, MotorState state);

    static MotorVelocityController ofProfile(Function<MotorState, Angle> profile, SetpointVelocityController setpointController) {
        return state -> setpointController.calculate(profile.apply(state), state);
    }
}
