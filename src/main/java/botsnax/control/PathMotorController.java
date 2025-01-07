package botsnax.control;

import edu.wpi.first.units.BaseUnits;
import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.DoubleFunction;
import java.util.function.Function;

import static edu.wpi.first.units.ImmutableMeasure.ofBaseUnits;

public class PathMotorController implements MotorController {
    private final Function<MotorState, Angle> profile;
    private final SetpointController outputController;

    private PathMotorController(Function<MotorState, Angle> profile, SetpointController outputController) {
        this.profile = profile;
        this.outputController = outputController;
    }

    public Voltage calculate(MotorState state) {
        return outputController.calculate(profile.apply(state), state);
    }

    public static PathMotorController ofPath(DoubleFunction<Double> path, SetpointController outputController) {
        Function<MotorState, Angle> profile = state -> BaseUnits.AngleUnit.of(path.apply(state.getTime().baseUnitMagnitude()));

        return new PathMotorController(profile, outputController);
    }
}
