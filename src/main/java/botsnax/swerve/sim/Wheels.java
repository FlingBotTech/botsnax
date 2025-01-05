package botsnax.swerve.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;

import java.util.function.Function;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.min;
import static java.lang.Math.pow;

public record Wheels(
        int wheelCount,
        Function<Integer, DCMotor> motorSupplier,
        double gearRatio,
        Measure<Distance> radius,
        Measure<Mult<Mass, Mult<Distance, Distance>>> wheelMoment,
        Measure<Current> currentLimit) {

    public Measure<Velocity<Angle>> getMaxSpeed() {
        DCMotor motor = motorSupplier.apply(wheelCount);

        return RadiansPerSecond.of(motor.freeSpeedRadPerSec).times(1.0 / gearRatio);
    }

    public Measure<Velocity<Angle>> getDampingRate(Measure<Mult<Mass, Mult<Distance, Distance>>> carriageMoment) {
        DCMotor motor = motorSupplier.apply(wheelCount);
        double maxTorque = motor.KtNMPerAmp * min(currentLimit.in(Amps), motor.stallCurrentAmps) * wheelCount;
        double backEmf = maxTorque / motor.freeSpeedRadPerSec;
        double totalMoment = wheelMoment.times(wheelCount).plus(carriageMoment).baseUnitMagnitude();

        return  RadiansPerSecond.of(pow(gearRatio, 2) * backEmf / totalMoment);
    }
}
