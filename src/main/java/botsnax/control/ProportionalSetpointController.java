package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import botsnax.system.motor.MotorState;

public class ProportionalSetpointController implements SetpointController {
    private final double gain;

    public ProportionalSetpointController(double gain) {
        this.gain = gain;
    }

    @Override
    public double calculate(Measure<Angle> setpoint, MotorState state) {
        return gain * setpoint.minus(state.getAngle()).baseUnitMagnitude();
    }

    public static ProportionalSetpointController create(
            double maximumVoltage,
            Measure<Angle> errorToApplyMaximumVoltage) {
        double gain = maximumVoltage / errorToApplyMaximumVoltage.baseUnitMagnitude();
        return new ProportionalSetpointController(gain);
    }
}
