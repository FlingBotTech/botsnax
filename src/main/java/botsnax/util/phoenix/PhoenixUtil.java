package botsnax.util.phoenix;

import botsnax.system.motor.phoenix.TalonFXCompoundMotor;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static java.lang.Math.abs;

public class PhoenixUtil {
    public static AngleUnit DEVICE_ANGLE_UNITS = Rotations;
    public static AngularVelocityUnit DEVICE_VELOCITY_UNITS = DEVICE_ANGLE_UNITS.per(Second);

    public static void validateSignalValue(StatusSignal<Angle> signal, double expectedValue) {
        StatusCode status = signal.waitForUpdate(1.0).getStatus();

        if (!status.isOK()) {
            throw new RuntimeException("Failed update signal: " + signal.getName());
        }

        double value = signal.getValue().baseUnitMagnitude();

        if (abs(value - expectedValue) > 0.01) {
            throw new RuntimeException("Failed to validate signal: " + signal.getName() + ", value = " + value + ", expectedValue = " + expectedValue);
        }
    }

    public static void setAndValidatePosition(CANcoder encoder, Angle position) {
        double positionValue = position.in(DEVICE_ANGLE_UNITS);
        StatusCode status = encoder.setPosition(positionValue);

        if (!status.isOK()) {
            throw new RuntimeException("Failed to set motor position: " + status);
        }

        validateSignalValue(encoder.getPosition(), positionValue);
    }

    public static void setAndValidatePosition(TalonFX motor, Angle position) {
        double positionValue = position.in(Rotations);
        StatusCode status = motor.setPosition(positionValue);

        if (!status.isOK()) {
            throw new RuntimeException("Failed to set motor position: " + status);
        }

        validateSignalValue(motor.getPosition(), positionValue);
    }

    public static void setAndValidatePosition(TalonFXCompoundMotor motor, Angle position) {
        PhoenixUtil.setAndValidatePosition(motor.getPrimary().get(), position);
    }
}
