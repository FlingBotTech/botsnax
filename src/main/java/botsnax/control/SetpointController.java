package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import botsnax.system.motor.MotorState;

import java.util.function.Function;

public interface SetpointController {
    double calculate(Measure<Angle> setpoint, MotorState state);

    static SetpointController combine(SetpointController setpointController, MotorController controller) {
        return (setpoint, state) -> setpointController.calculate(setpoint, state) + controller.calculate(state);
    }

    static SetpointController ofVelocityController(SetpointVelocityController velocityController, MotorKinematics kinematics) {
        return (setpoint, state) -> kinematics.getVoltageForVelocity(velocityController.calculate(setpoint, state), state);
    }

    static MotorController ofProfile(Function<MotorState, Measure<Angle>> profile, SetpointController setpointController) {
        return state -> setpointController.calculate(profile.apply(state), state);
    }

    static MotorController ofSetpoint(Measure<Angle> setpoint, SetpointController setpointController) {
        return ofProfile(state -> setpoint, setpointController);
    }
}
