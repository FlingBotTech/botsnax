package botsnax.turret.calibrate;

import botsnax.commands.calibrate.VelocityCalibration;

import java.util.Optional;

public class TurretCalibration {
    public static final String PREFERENCE_BASE_NAME = "Turret";

    public static Optional<VelocityCalibration> load() {
        return VelocityCalibration.load(PREFERENCE_BASE_NAME);
    }
}
