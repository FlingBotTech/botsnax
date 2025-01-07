package botsnax.control;

import edu.wpi.first.units.*;
import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.*;
import static java.lang.Math.abs;

public class ReactionTimeVelocityController implements SetpointVelocityController {
    private final Time reactionTime;
    private final AngularVelocity minVelocity;
    private final AngularVelocity maxVelocity;
    private final Angle deadband;

    public ReactionTimeVelocityController(Time reactionTime, AngularVelocity maxVelocity, Angle deadband) {
        this.reactionTime = reactionTime;
        this.minVelocity = Rotations.per(Second).of(0);
        this.maxVelocity = maxVelocity;
        this.deadband = deadband;
    }

    public ReactionTimeVelocityController(Time reactionTime, AngularVelocity minVelocity, AngularVelocity maxVelocity, Angle deadband) {
        this.reactionTime = reactionTime;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.deadband = deadband;
    }

    @Override
    public AngularVelocity calculate(Angle setpoint, MotorState state) {
        return calculate(setpoint.minus(state.getAngle()));
    }

    public AngularVelocity calculate(Angle error) {
        if (abs(error.baseUnitMagnitude()) <= deadband.baseUnitMagnitude()) {
            return RadiansPerSecond.of(0);
        } else {
            AngularVelocity reactionVelocity = BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(
                    error.baseUnitMagnitude() / reactionTime.baseUnitMagnitude());
            AngularVelocity velocity = BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(
                    signum(reactionVelocity.baseUnitMagnitude()) *
                            min(max(abs(reactionVelocity.baseUnitMagnitude()), minVelocity.baseUnitMagnitude()), maxVelocity.baseUnitMagnitude()));

            return velocity;
        }
    }
}
