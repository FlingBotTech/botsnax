package botsnax.swerve.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

import java.util.function.Function;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.min;
import static java.lang.Math.pow;

public record Wheels(
        int wheelCount,
        Function<Integer, DCMotor> motorSupplier,
        double gearRatio,
        Distance radius,
        MomentOfInertia wheelMoment,
        Current currentLimit) {

    public AngularVelocity getMaxSpeed() {
        DCMotor motor = motorSupplier.apply(wheelCount);

        return RadiansPerSecond.of(motor.freeSpeedRadPerSec).times(1.0 / gearRatio);
    }

    public AngularVelocity getDampingRate(MomentOfInertia carriageMoment) {
        DCMotor motor = motorSupplier.apply(wheelCount);
        double maxTorque = motor.KtNMPerAmp * min(currentLimit.in(Amps), motor.stallCurrentAmps) * wheelCount;
        double backEmf = maxTorque / motor.freeSpeedRadPerSec;
        double totalMoment = wheelMoment.times(wheelCount).plus(carriageMoment).baseUnitMagnitude();

        return  RadiansPerSecond.of(pow(gearRatio, 2) * backEmf / totalMoment);
    }
}
