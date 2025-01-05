package botsnax.control;

import botsnax.system.motor.MotorState;

public interface MotorController {
    double calculate(MotorState state);

    static MotorController ofVelocityController(MotorVelocityController velocityController, MotorKinematics kinematics) {
        return state -> kinematics.getVoltageForVelocity(velocityController.calculate(state), state);
    }
}
