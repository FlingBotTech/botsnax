package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import botsnax.system.motor.MotorState;

import java.util.function.DoubleFunction;

import static edu.wpi.first.units.ImmutableMeasure.ofBaseUnits;

public class PathMotorController implements MotorController {
    private final MotorController profile;
    private final SetpointController outputController;

    private PathMotorController(MotorController profile, SetpointController outputController) {
        this.profile = profile;
        this.outputController = outputController;
    }

    public double calculate(MotorState state) {
        Measure<Angle> setpoint = ofBaseUnits(profile.calculate(state), BaseUnits.Angle);
        return outputController.calculate(setpoint, state);
    }

    public static PathMotorController ofPath(DoubleFunction<Double> path, SetpointController outputController) {
        MotorController profile = state -> path.apply(state.getTime().baseUnitMagnitude());

        return new PathMotorController(profile, outputController);
    }
}
