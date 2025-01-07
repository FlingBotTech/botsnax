package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.Function;

public interface SetpointController {
    Voltage calculate(Angle setpoint, MotorState state);

    static SetpointController combine(SetpointController setpointController, MotorController controller) {
        return (setpoint, state) -> setpointController.calculate(setpoint, state).plus(controller.calculate(state));
    }

    static SetpointController ofVelocityController(SetpointVelocityController velocityController, MotorKinematics kinematics) {
        return (setpoint, state) -> kinematics.getVoltageForVelocity(velocityController.calculate(setpoint, state), state);
    }

    static MotorController ofProfile(Function<MotorState, Angle> profile, SetpointController setpointController) {
        return state -> setpointController.calculate(profile.apply(state), state);
    }

    static MotorController ofSetpoint(Angle setpoint, SetpointController setpointController) {
        return ofProfile(state -> setpoint, setpointController);
    }
}
