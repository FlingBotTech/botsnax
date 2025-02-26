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

public class VelocityCalibration implements MotorKinematics {
    private final double velocityToVoltageSlope;
    private final Voltage minimumVoltage;

    public double getVelocityToVoltageSlope() {
        return velocityToVoltageSlope;
    }

    public VelocityCalibration(double velocityToVoltageSlope, Voltage minimumVoltage) {
        double minimumVoltageV = minimumVoltage.in(Volts);

        if (minimumVoltageV < 0) {
            if (minimumVoltageV > -0.1) {
                new RuntimeException("Warning: minimum voltage is negative, clamping to zero: " + minimumVoltageV)
                    .printStackTrace(System.err);
                this.minimumVoltage = Volts.zero();
            }
            else {
                throw new RuntimeException("Minimum voltage is negative: " + minimumVoltageV);
            }
        } else {
            this.minimumVoltage = minimumVoltage;
        }

        if (velocityToVoltageSlope < 0) {
            throw new RuntimeException("velocityToVoltageSlope is negative: " + velocityToVoltageSlope);
        } else {
            this.velocityToVoltageSlope = velocityToVoltageSlope;
        }
    }

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

    public Slot0Configs getTalonFXSlot0Configs() {
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
