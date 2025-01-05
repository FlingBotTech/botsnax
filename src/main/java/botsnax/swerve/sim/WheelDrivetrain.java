package botsnax.swerve.sim;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.Seconds;
import static java.lang.Math.*;

public record WheelDrivetrain(
        Wheels wheels,
        Measure<Mass> carriageMass,
        Measure<Mult<Mass, Mult<Distance, Distance>>> carriageMoment,
        double frictionCoefficient) {

   private Measure<Velocity<Angle>> getAdjustedFinalVelocity(Measure<Velocity<Angle>> finalVelocity, Measure<Velocity<Angle>> currentVelocity) {
       if ((currentVelocity.baseUnitMagnitude() != 0) && signum(finalVelocity.baseUnitMagnitude()) != signum(currentVelocity.baseUnitMagnitude())) {
           double opposingMagnitude = carriageMass.baseUnitMagnitude() * frictionCoefficient;
           return finalVelocity.minus(BaseUnits.Angle.per(BaseUnits.Time).of(
                   signum(currentVelocity.magnitude()) * opposingMagnitude));
       } else {
           return finalVelocity;
       }
   }

    public Measure<Velocity<Angle>> getVelocityChange(Measure<Velocity<Angle>> finalVelocity, Measure<Velocity<Angle>> currentVelocity, Measure<Time> dt) {
        Measure<Velocity<Angle>> adjustedFinalVelocity = getAdjustedFinalVelocity(finalVelocity, currentVelocity);
        Measure<Velocity<Angle>> dampingRate = wheels.getDampingRate(carriageMoment);

        Measure<Velocity<Angle>> deltaV = currentVelocity.minus(adjustedFinalVelocity).times(exp(-1 * dampingRate.times(dt).baseUnitMagnitude()) - 1);

        if ((finalVelocity.magnitude() == 0) && (abs(deltaV.baseUnitMagnitude()) > abs(currentVelocity.baseUnitMagnitude()))) {
            return currentVelocity.times(-1);
        } else {
            return deltaV;
        }
    }

    @SuppressWarnings("unchecked")
    public Measure<Angle> getAngleChange(Measure<Velocity<Angle>> finalVelocity, Measure<Velocity<Angle>> currentVelocity, Measure<Time> dt) {
        Measure<Velocity<Angle>> adjustedFinalVelocity = getAdjustedFinalVelocity(finalVelocity, currentVelocity);
        Measure<Velocity<Angle>> dampingRate = wheels.getDampingRate(carriageMoment);
        Measure<Time> multiplier = Seconds.of(-1.0 / dampingRate.baseUnitMagnitude())
                .times(exp(-1 * dampingRate.times(dt).baseUnitMagnitude()) - 1);
        Measure<Angle> d1 = (Measure<Angle>) adjustedFinalVelocity.times(dt);
        Measure<Angle> d2 = (Measure<Angle>) currentVelocity.minus(adjustedFinalVelocity).times(multiplier);

        return d1.plus(d2);
    }
}
