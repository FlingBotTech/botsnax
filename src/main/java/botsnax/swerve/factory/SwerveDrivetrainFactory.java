package botsnax.swerve.factory;

import botsnax.swerve.SwerveCalibration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import botsnax.swerve.SwerveDrivetrain;
import botsnax.swerve.SwerveModule;
import botsnax.swerve.SwerveOdometryUpdater;
import botsnax.system.Gyro;
import botsnax.util.PeriodicUpdater;

import static java.util.Arrays.stream;

public abstract class SwerveDrivetrainFactory {
    protected final SwerveDrivetrainConstants drivetrainConstants;
    protected final SwerveModuleConstants[] moduleConstants;

    protected SwerveDrivetrainFactory(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants ... moduleConstants) {
        this.drivetrainConstants = drivetrainConstants;
        this.moduleConstants = moduleConstants;
    }

    public SwerveDrivetrain create() {
        SwerveCalibration.apply(moduleConstants);

        Gyro gyro = createGyro();

        SwerveModule[] modules = stream(moduleConstants)
                .map(this :: createModule)
                .toArray(SwerveModule[] :: new);

        Translation2d[] locations = stream(moduleConstants)
                .map(c -> new Translation2d(c.LocationX, c.LocationY))
                .toArray(Translation2d[] :: new);

        SwerveModulePosition[] positions = stream(modules)
                .map(this :: getPosition)
                .toArray(SwerveModulePosition[] :: new);

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations);

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyro.getHeading(),
                positions,
                new Pose2d()
        );

        SwerveOdometryUpdater odometryUpdater = new SwerveOdometryUpdater(
                gyro,
                modules,
                this :: getPosition,
                poseEstimator
        );

        PeriodicUpdater<SwerveOdometryUpdater> periodicUpdater = createPeriodicUpdater(odometryUpdater, gyro, modules);

        periodicUpdater.start();

        return new SwerveDrivetrain(kinematics, modules, periodicUpdater);
    }

    protected abstract SwerveModulePosition getPosition(SwerveModule module);
    protected abstract Gyro createGyro();
    protected abstract SwerveModule createModule(SwerveModuleConstants moduleConstants);
    protected abstract PeriodicUpdater<SwerveOdometryUpdater> createPeriodicUpdater(SwerveOdometryUpdater odometryUpdater, Gyro gyro, SwerveModule[] modules);
}
