package botsnax.swerve;

import botsnax.commands.calibrate.VelocityCalibration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class DriveMotorsCalibration {
    public static void apply(VelocityCalibration calibration, SwerveModuleConstants<?, ?, ?> constants) {
        calibration.applyTalonFXSlot0Configs(constants.DriveMotorGains);
    }

    public static void apply(SwerveModuleConstants<?, ?, ?>[] constants) {
        VelocityCalibration[] calibrations = new VelocityCalibration[constants.length];
        boolean allLoaded = true;

        for (int i = 0; i < constants.length; i++) {
            calibrations[i] = VelocityCalibration.load(SwerveCalibration.getDriveVelocityBaseName(i)).orElse(null);

            if (calibrations[i] == null) {
                System.err.println("Swerve module " + i + " is missing velocity calibration");
                allLoaded = false;
            }
        }

        if (allLoaded) {
            for (int i = 0; i < constants.length; i++) {
                apply(calibrations[i], constants[i]);
            }
        }
    }
}
