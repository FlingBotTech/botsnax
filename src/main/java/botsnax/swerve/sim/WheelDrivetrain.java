package botsnax.swerve.sim;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Seconds;
import static java.lang.Math.*;

public record WheelDrivetrain(
        Wheels wheels,
        Mass carriageMass,
        MomentOfInertia carriageMoment,
        double frictionCoefficient) {

   private AngularVelocity getAdjustedFinalVelocity(AngularVelocity finalVelocity, AngularVelocity currentVelocity) {
       if ((currentVelocity.baseUnitMagnitude() != 0) && signum(finalVelocity.baseUnitMagnitude()) != signum(currentVelocity.baseUnitMagnitude())) {
           double opposingMagnitude = carriageMass.baseUnitMagnitude() * frictionCoefficient;
           return finalVelocity.minus(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(
                   signum(currentVelocity.magnitude()) * opposingMagnitude));
       } else {
           return finalVelocity;
       }
   }

    public AngularVelocity getVelocityChange(AngularVelocity finalVelocity, AngularVelocity currentVelocity, Time dt) {
        AngularVelocity adjustedFinalVelocity = getAdjustedFinalVelocity(finalVelocity, currentVelocity);
        AngularVelocity dampingRate = wheels.getDampingRate(carriageMoment);

        AngularVelocity deltaV = currentVelocity.minus(adjustedFinalVelocity).times(exp(-1 * dampingRate.times(dt).baseUnitMagnitude()) - 1);

        if ((finalVelocity.magnitude() == 0) && (abs(deltaV.baseUnitMagnitude()) > abs(currentVelocity.baseUnitMagnitude()))) {
            return currentVelocity.times(-1);
        } else {
            return deltaV;
        }
    }

    public Angle getAngleChange(AngularVelocity finalVelocity, AngularVelocity currentVelocity, Time dt) {
        AngularVelocity adjustedFinalVelocity = getAdjustedFinalVelocity(finalVelocity, currentVelocity);
        AngularVelocity dampingRate = wheels.getDampingRate(carriageMoment);
        Time multiplier = Seconds.of(-1.0 / dampingRate.baseUnitMagnitude())
                .times(exp(-1 * dampingRate.times(dt).baseUnitMagnitude()) - 1);
        Angle d1 = adjustedFinalVelocity.times(dt);
        Angle d2 = currentVelocity.minus(adjustedFinalVelocity).times(multiplier);

        return d1.plus(d2);
    }
}
