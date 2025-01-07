package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Voltage;

import java.util.Arrays;

public class CompositeMotorController implements MotorController {
    private final MotorController[] motorControllers;

    public CompositeMotorController(MotorController ... motorControllers) {
        this.motorControllers = motorControllers;
    }

    @Override
    public Voltage calculate(MotorState state) {
        return BaseUnits.VoltageUnit.of(
                Arrays.stream(motorControllers)
                .mapToDouble(c -> c.calculate(state).baseUnitMagnitude())
                .sum());
    }
}
