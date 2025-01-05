package botsnax.control;

import edu.wpi.first.units.*;
import botsnax.system.motor.MotorState;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.*;
import static java.lang.Math.abs;

public class ReactionTimeVelocityController implements SetpointVelocityController {
    private final Measure<Time> reactionTime;
    private final Measure<Velocity<Angle>> minVelocity;
    private final Measure<Velocity<Angle>> maxVelocity;
    private final Measure<Angle> deadband;

    public ReactionTimeVelocityController(Measure<Time> reactionTime, Measure<Velocity<Angle>> maxVelocity, Measure<Angle> deadband) {
        this.reactionTime = reactionTime;
        this.minVelocity = Rotations.per(Second).of(0);
        this.maxVelocity = maxVelocity;
        this.deadband = deadband;
    }

    public ReactionTimeVelocityController(Measure<Time> reactionTime, Measure<Velocity<Angle>> minVelocity, Measure<Velocity<Angle>> maxVelocity, Measure<Angle> deadband) {
        this.reactionTime = reactionTime;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.deadband = deadband;
    }

    @Override
    public Measure<Velocity<Angle>> calculate(Measure<Angle> setpoint, MotorState state) {
        return calculate(setpoint.minus(state.getAngle()));
    }

    public Measure<Velocity<Angle>> calculate(Measure<Angle> error) {
        if (abs(error.baseUnitMagnitude()) <= deadband.baseUnitMagnitude()) {
            return RadiansPerSecond.of(0);
        } else {
            Measure<Velocity<Angle>> reactionVelocity = BaseUnits.Angle.per(BaseUnits.Time).of(
                    error.baseUnitMagnitude() / reactionTime.baseUnitMagnitude());
            Measure<Velocity<Angle>> velocity = BaseUnits.Angle.per(BaseUnits.Time).of(
                    signum(reactionVelocity.baseUnitMagnitude()) *
                            min(max(abs(reactionVelocity.baseUnitMagnitude()), minVelocity.baseUnitMagnitude()), maxVelocity.baseUnitMagnitude()));

            return velocity;
        }
    }
}
