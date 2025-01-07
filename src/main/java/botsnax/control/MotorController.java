package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Voltage;

public interface MotorController {
    Voltage calculate(MotorState state);

    static MotorController ofVelocityController(MotorVelocityController velocityController, MotorKinematics kinematics) {
        return state -> kinematics.getVoltageForVelocity(velocityController.calculate(state), state);
    }
}
