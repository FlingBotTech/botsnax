package botsnax.util.phoenix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import botsnax.system.motor.phoenix.TalonFXCompoundMotor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static java.lang.Math.abs;

public class PhoenixUtil {
    public static Unit<Angle> DEVICE_ANGLE_UNITS = Rotations;
    public static Unit<Velocity<Angle>> DEVICE_VELOCITY_UNITS = DEVICE_ANGLE_UNITS.per(Second);

    public static void validateSignalValue(StatusSignal<Double> signal, double expectedValue) {
        StatusCode status = signal.waitForUpdate(1.0).getStatus();

        if (!status.isOK()) {
            throw new RuntimeException("Failed update signal: " + signal.getName());
        }

        double value = signal.getValue();

        if (abs(value - expectedValue) > 0.01) {
            throw new RuntimeException("Failed to validate signal: " + signal.getName() + ", value = " + value + ", expectedValue = " + expectedValue);
        }
    }

    public static void setAndValidatePosition(CANcoder encoder, Measure<Angle> position) {
        double positionValue = position.in(DEVICE_ANGLE_UNITS);
        StatusCode status = encoder.setPosition(positionValue);

        if (!status.isOK()) {
            throw new RuntimeException("Failed to set motor position: " + status);
        }

        validateSignalValue(encoder.getPosition(), positionValue);
    }

    public static void setAndValidatePosition(TalonFX motor, Measure<Angle> position) {
        double positionValue = position.in(Rotations);
        StatusCode status = motor.setPosition(positionValue);

        if (!status.isOK()) {
            throw new RuntimeException("Failed to set motor position: " + status);
        }

        validateSignalValue(motor.getPosition(), positionValue);
    }

    public static void setAndValidatePosition(TalonFXCompoundMotor motor, Measure<Angle> position) {
        PhoenixUtil.setAndValidatePosition(motor.getPrimary().get(), position);
    }
}
