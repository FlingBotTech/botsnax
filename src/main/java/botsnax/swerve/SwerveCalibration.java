package botsnax.swerve;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import static edu.wpi.first.wpilibj.Preferences.getDouble;
import static java.lang.Double.NaN;

public class SwerveCalibration {
    public static final String PREFERENCE_BASE_NAME = "SwerveModule";
    public static final String VELOCITY_BASE_NAME = "drive.velocity.";
    public static final String OFFSET_BASE_NAME = "encoder.offset";
    public static final String COUPLING_RATIO_BASE_NAME = "coupling_ratio.";

    private static String getModuleBaseName(int i) {
        return PREFERENCE_BASE_NAME + "." + i + ".";
    }

    public static String getOffsetName(int i) {
        return getModuleBaseName(i) + OFFSET_BASE_NAME;
    }

    public static String getCouplingRatioName(int i) {
        return getModuleBaseName(i) + COUPLING_RATIO_BASE_NAME;
    }

    public static String getDriveVelocityBaseName(int i) {
        return getModuleBaseName(i) + VELOCITY_BASE_NAME;
    }

    public static void applyOffset(int index, SwerveModuleConstants<?, ?, ?> constants) {
        double offset = getDouble(getOffsetName(index), NaN);

        if (!Double.isNaN(offset)) {
            constants.withEncoderOffset(offset);
        } else {
            System.err.println("Swerve module " + index + " is missing encoder offset value");
        }
    }

    public static void apply(SwerveModuleConstants<?, ?, ?>[] constants) {
        DriveMotorsCalibration.apply(constants);

        for (int i = 0; i < constants.length; i++) {
            applyOffset(i, constants[i]);
        }
    }
}
