package botsnax.swerve.phoenix;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.units.*;
import botsnax.commands.calibrate.VelocityCalibration;
import botsnax.util.phoenix.CANcoderUtil;
import botsnax.swerve.sim.SwerveSim;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class PhoenixSwerveDrivetrainBuilder {
    private static final String CALIBRATION_BASE_NAME = "SwerveModule";
    private static final double DRIVE_KP = 0;

    private final SwerveDrivetrainConstants drivetrainConstants;
    private final SwerveModuleConstants[] moduleConstants;

    private Optional<PhoenixSwerveDrivetrain> drivetrainIfAny = Optional.empty();

    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return drivetrainConstants;
    }

    public SwerveModuleConstants[] getModuleConstants() {
        return moduleConstants;
    }

    public PhoenixSwerveDrivetrainBuilder(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... moduleConstants) {
        this.drivetrainConstants = driveTrainConstants;
        this.moduleConstants = moduleConstants;
    }

    public Optional<SwerveDrivetrain> createDrivetrain() {
        SwerveModuleConstants[] modulesWithOffsets = Arrays
                .stream(moduleConstants)
                .flatMap(module -> loadOffset(module).stream())
                .map(this::loadVelocityCalibration)
                .toArray(SwerveModuleConstants[]::new);

        if (modulesWithOffsets.length == moduleConstants.length) {
            PhoenixSwerveDrivetrain drivetrain = new PhoenixSwerveDrivetrain(drivetrainConstants, modulesWithOffsets);
            drivetrainIfAny = Optional.of(drivetrain);

            return Optional.of(drivetrain);
        } else {
            System.err.println("Encoder offsets not loaded for swerve modules");
            return Optional.empty();
        }
    }

    private Stream<Measure<Velocity<Distance>>> getModuleLinearVelocities() {
        return Arrays
                .stream(moduleConstants)
                .flatMap(module -> VelocityCalibration
                        .load(getVelocityCalibrationPreferenceName(module.DriveMotorId))
                        .map(calibration -> new VelocityConversion(module, calibration
                                .getVelocityForVoltage(12))
                                .toLinearVelocity())
                        .stream());
    }

    public Optional<Measure<Velocity<Distance>>> getCalibratedVelocityAt12v() {
        if (getModuleLinearVelocities().count() == moduleConstants.length) {
            return getModuleLinearVelocities().min(Comparator.naturalOrder());
        } else {
            System.err.println("Swerve module velocities not calibrated");
            return Optional.empty();
        }
    }

    public Measure<Velocity<Distance>> getConfiguredVelocityAt12v() {
        return MetersPerSecond.of(
                Arrays.stream(moduleConstants).mapToDouble(module -> module.SpeedAt12VoltsMps).min().orElseThrow()
        );
    }

    public Optional<SwerveSim> startSim(Function<PhoenixSwerveDrivetrain, SwerveSim> start) {
        return Utils.isSimulation() ? drivetrainIfAny.map(start) : Optional.empty();
    }

    public static String getEncoderOffsetPreferenceName(int encoderId) {
        return CALIBRATION_BASE_NAME + ".offset_" + encoderId;
    }

    public static String getVelocityCalibrationPreferenceName(int motorId) {
        return CALIBRATION_BASE_NAME + ".velocity_" + motorId;
    }

    private Optional<SwerveModuleConstants> loadOffset(SwerveModuleConstants module) {
        return CANcoderUtil
                .loadOffset(getEncoderOffsetPreferenceName(module.CANcoderId))
                .map(module::withCANcoderOffset);
    }

    private SwerveModuleConstants loadVelocityCalibration(SwerveModuleConstants module) {
        return VelocityCalibration
                .load(getVelocityCalibrationPreferenceName(module.DriveMotorId))
                .map(calibration -> module.withDriveMotorGains(calibration.asTalonConfig().withKP(DRIVE_KP)))
                .orElse(module);
    }
}
