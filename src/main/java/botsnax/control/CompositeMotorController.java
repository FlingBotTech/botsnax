package botsnax.control;

import botsnax.system.motor.MotorState;

import java.util.Arrays;

public class CompositeMotorController implements MotorController {
    private final MotorController[] motorControllers;

    public CompositeMotorController(MotorController ... motorControllers) {
        this.motorControllers = motorControllers;
    }

    @Override
    public double calculate(MotorState state) {
        return Arrays.stream(motorControllers)
                .mapToDouble(c -> c.calculate(state))
                .sum();
    }
}
