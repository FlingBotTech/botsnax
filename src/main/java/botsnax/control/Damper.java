package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import botsnax.system.motor.MotorState;

import static edu.wpi.first.units.Units.Seconds;

public class Damper implements MotorController {
    private final double gain;
    private final Measure<Velocity<Angle>> velocityPerVolt;

    public Damper(double gain, Measure<Velocity<Angle>> velocityPerVolt) {
        this.gain = gain;
        this.velocityPerVolt = velocityPerVolt;
    }

    @Override
    public double calculate(MotorState state) {
        double delta = -1 * state.getVelocity().in(velocityPerVolt.unit()) * gain / velocityPerVolt.magnitude();
        return delta - state.getVoltage();
    }

    public static Damper create(Measure<Time> halfLife, Measure<Velocity<Angle>> velocityPerVolt, Measure<Time> timeIncrement) {
        double rate = Math.log(0.5) / halfLife.in(Seconds);
        double discretized = Math.exp(rate * timeIncrement.in(Seconds));

        return new Damper(discretized, velocityPerVolt);
    }
}
