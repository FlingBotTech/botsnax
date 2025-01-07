package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Damper implements MotorController {
    private final double gain;
    private final AngularVelocity velocityPerVolt;

    public Damper(double gain, AngularVelocity velocityPerVolt) {
        this.gain = gain;
        this.velocityPerVolt = velocityPerVolt;
    }

    @Override
    public Voltage calculate(MotorState state) {
        Voltage delta = Volts.of(-1 * state.getVelocity().in(velocityPerVolt.unit()) * gain / velocityPerVolt.magnitude());
        return delta.minus(state.getVoltage());
    }

    public static Damper create(Time halfLife, AngularVelocity velocityPerVolt, Time timeIncrement) {
        double rate = Math.log(0.5) / halfLife.in(Seconds);
        double discretized = Math.exp(rate * timeIncrement.in(Seconds));

        return new Damper(discretized, velocityPerVolt);
    }
}
