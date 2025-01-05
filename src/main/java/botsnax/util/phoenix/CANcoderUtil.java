package botsnax.util.phoenix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Preferences;

import java.util.Optional;

import static edu.wpi.first.units.Units.Rotations;
import static java.lang.Double.NaN;

public class CANcoderUtil {
    private static MagnetSensorConfigs getConfig(CANcoder cancoder) {
        MagnetSensorConfigs configs = new MagnetSensorConfigs();
        CANcoderConfigurator configurator = cancoder.getConfigurator();
        StatusCode statusCode = configurator.refresh(configs, 10);

        handleStatus(statusCode);

        return configs;
    }

    private static StatusSignal<Double> getSignal(StatusSignal<Double> signal) {
        StatusSignal<Double> refreshed = signal.waitForUpdate(1, true);
        return refreshed;
    }

    public static Measure<Angle> getOffset(CANcoder cancoder) {
        return Rotations.of(getConfig(cancoder).MagnetOffset);
    }

    public static void setOffset(CANcoder cancoder, double value) {
        MagnetSensorConfigs configs = new MagnetSensorConfigs().withMagnetOffset(value);
        CANcoderConfigurator configurator = cancoder.getConfigurator();
        StatusCode statusCode = configurator.apply(configs, 1);
        handleStatus(statusCode);
    }

    private static void handleStatus(StatusCode statusCode) {
        if (!statusCode.isOK()) {
            System.err.println("CANcoder Error: " + statusCode);
            throw new RuntimeException("Operation failed on CANcoder, status: " + statusCode);
        }
    }

    public static double getOffsetToZeroCurrentPosition(CANcoder cancoder) {
        double position = getSignal(cancoder.getPosition()).getValue();
        MagnetSensorConfigs config = getConfig(cancoder);
        double netPosition = position - config.MagnetOffset;

        return -netPosition;
    }

    public static void saveOffsetToZeroCurrentPosition(String name, CANcoder cancoder) {
        double offset = getOffsetToZeroCurrentPosition(cancoder);
        Preferences.setDouble(name, offset);
    }

    public static Optional<Double> loadOffset(String name) {
        double value = Preferences.getDouble(name, NaN);

        if (!Double.isNaN(value)) {
            return Optional.of(value);
        } else {
            return Optional.empty();
        }
    }
}
