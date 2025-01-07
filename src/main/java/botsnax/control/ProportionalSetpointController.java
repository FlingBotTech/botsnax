package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class ProportionalSetpointController implements SetpointController {
    private final Voltage gain;

    public ProportionalSetpointController(Voltage gain) {
        this.gain = gain;
    }

    @Override
    public Voltage calculate(Angle setpoint, MotorState state) {
        return gain.times(setpoint.minus(state.getAngle()).baseUnitMagnitude());
    }

    public static ProportionalSetpointController create(
            Voltage maximumVoltage,
            Angle errorToApplyMaximumVoltage) {
        Voltage gain = maximumVoltage.div(errorToApplyMaximumVoltage.baseUnitMagnitude());
        return new ProportionalSetpointController(gain);
    }
}
