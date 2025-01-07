package botsnax.commands.calibrate;

import botsnax.control.MotorKinematics;
import botsnax.system.motor.MotorState;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

import java.util.Optional;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static java.lang.Double.NaN;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public record VelocityCalibration (double velocityToVoltageSlope, Voltage minimumVoltage) implements MotorKinematics {
    @Override
    public Voltage getVoltageForVelocity(AngularVelocity velocity, MotorState state) {
        double velocityMagnitude = velocity.baseUnitMagnitude();
        double absoluteVoltage = (abs(velocityMagnitude) * velocityToVoltageSlope) + minimumVoltage.baseUnitMagnitude();

        return Volts.of(signum(velocityMagnitude) * absoluteVoltage);
    }

    public AngularVelocity getVelocityForVoltage(Voltage voltage) {
        return (BaseUnits.AngleUnit.per(BaseUnits.TimeUnit)).of(voltage.minus(minimumVoltage).baseUnitMagnitude() / velocityToVoltageSlope);
    }

    public void save(String baseName) {
        Preferences.setDouble(baseName + ".FF.kv", velocityToVoltageSlope);
        Preferences.setDouble(baseName + ".FF.ks", minimumVoltage.baseUnitMagnitude());
    }

    public Slot0Configs asTalonConfig() {
        return new Slot0Configs()
                .withKS(minimumVoltage.baseUnitMagnitude())
                .withKV(1.0 / (BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(1.0 / velocityToVoltageSlope).in(RotationsPerSecond)));
    }

    public static Optional<VelocityCalibration> load(String baseName) {
        double kv = Preferences.getDouble(baseName + ".FF.kv", NaN);
        double ks = Preferences.getDouble(baseName + ".FF.ks", NaN);

        if (!Double.isNaN(kv) && !Double.isNaN(ks)) {
            return Optional.of(new VelocityCalibration(kv, Volts.of(ks)));
        } else {
            return Optional.empty();
        }
    }
}
