package botsnax.commands.calibrate;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Preferences;
import botsnax.control.MotorKinematics;
import botsnax.system.motor.MotorState;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static java.lang.Double.NaN;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public record VelocityCalibration (double velocityToVoltageSlope, double minimumVoltage) implements MotorKinematics {
    @Override
    public double getVoltageForVelocity(Measure<Velocity<Angle>> velocity, MotorState state) {
        double velocityMagnitude = velocity.baseUnitMagnitude();
        double absoluteVoltage = (abs(velocityMagnitude) * velocityToVoltageSlope) + minimumVoltage;

        return signum(velocityMagnitude) * absoluteVoltage;
    }

    public Measure<Velocity<Angle>> getVelocityForVoltage(double voltage) {
        return (BaseUnits.Angle.per(BaseUnits.Time)).of((voltage - minimumVoltage) / velocityToVoltageSlope);
    }

    public void save(String baseName) {
        Preferences.setDouble(baseName + ".FF.kv", velocityToVoltageSlope);
        Preferences.setDouble(baseName + ".FF.ks", minimumVoltage);
    }

    public Slot0Configs asTalonConfig() {
        return new Slot0Configs()
                .withKS(minimumVoltage)
                .withKV(1.0 / (BaseUnits.Angle.per(BaseUnits.Time).of(1.0 / velocityToVoltageSlope).in(RotationsPerSecond)));
    }

    public static Optional<VelocityCalibration> load(String baseName) {
        double kv = Preferences.getDouble(baseName + ".FF.kv", NaN);
        double ks = Preferences.getDouble(baseName + ".FF.ks", NaN);

        if (!Double.isNaN(kv) && !Double.isNaN(ks)) {
            return Optional.of(new VelocityCalibration(kv, ks));
        } else {
            return Optional.empty();
        }
    }
}
