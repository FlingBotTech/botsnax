package botsnax.swerve.factory;

import botsnax.swerve.*;
import botsnax.swerve.listener.PoseLogger;
import botsnax.swerve.sim.GenericSwerveSim;
import botsnax.swerve.sim.IdealizedSwerveDrivetrain;
import botsnax.swerve.sim.IdealizedSwerveSim;
import botsnax.system.Gyro;
import botsnax.util.PeriodicUpdater;
import botsnax.util.SimRunner;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Mass;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;
import static java.util.Arrays.stream;

public abstract class GenericSwerveDrivetrainFactory {
    protected final Mass mass;
    protected final SwerveDrivetrainConstants drivetrainConstants;
    protected final SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    protected GenericSwerveDrivetrainFactory(Mass mass, SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?> ... moduleConstants) {
        this.mass = mass;
        this.drivetrainConstants = drivetrainConstants;
        this.moduleConstants = moduleConstants;
    }

    public SwerveController create() {
        SwerveCalibration.apply(moduleConstants);

        Gyro gyro = createGyro();

        SwerveModule[] modules = stream(moduleConstants)
                .map(this::createModule)
                .toArray(SwerveModule[]::new);

        Translation2d[] locations = stream(moduleConstants)
                .map(c -> new Translation2d(c.LocationX, c.LocationY))
                .toArray(Translation2d[]::new);

        SwerveModulePosition[] positions = stream(modules)
                .map(this::getPosition)
                .toArray(SwerveModulePosition[]::new);

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations);

        Pose2d initialPose = new Pose2d();

        IdealizedSwerveDrivetrain idealizedSwerveDrivetrain = IdealizedSwerveDrivetrain.ofRectangularChassis(
                moduleConstants,
                modules[0].getDrive().getDCMotor(),
                mass);

        IdealizedSwerveSim idealizedSwerveSim = idealizedSwerveDrivetrain.createSim(initialPose);
        GenericSwerveSim genericSwerveSim = new GenericSwerveSim(
                idealizedSwerveSim,
                gyro,
                modules,
                kinematics,
                moduleConstants
        );

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyro.getHeading(),
                positions,
                initialPose
        );

        PoseLogger poseLogger = new PoseLogger();

        SwerveOdometryUpdater odometryUpdater = new SwerveOdometryUpdater(
                gyro,
                modules,
                this::getPosition,
                pose -> {
                    if (isSimulation()) {
                        idealizedSwerveSim.setPose(pose);
                    } else {
                        poseEstimator.resetPose(pose);
                    }
                },
                (heading, modulePositions) -> {
                    if (isSimulation()) {
                        return idealizedSwerveSim.getPose();
                    } else {
                        poseEstimator.update(heading, modulePositions);
                        return poseEstimator.getEstimatedPosition();
                    }
                },
                poseLogger
        );

        PeriodicUpdater<SwerveOdometryUpdater> periodicUpdater = createPeriodicUpdater(odometryUpdater, gyro, modules);

        periodicUpdater.start();

        if (isSimulation()) {
            new SimRunner().start(
                    dt -> genericSwerveSim.update(dt, Volts.of(getBatteryVoltage()), modules),
                    Milliseconds.of(5));
        }

        GenericSwerveDrivetrain drivetrain = new GenericSwerveDrivetrain(kinematics, modules);
        SwerveModule.ApplyMode defaultApplyMode = getDefaultApplyode();

        return new SwerveController() {
            @Override
            public GenericSwerveDrivetrain getDrivetrain() {
                return drivetrain;
            }

            @Override
            public Pose2d getPose() {
                return poseLogger.getPose();
            }

            @Override
            public void setPose(Pose2d pose) {
                periodicUpdater.apply(odometryUpdater -> odometryUpdater.setPose(pose));
            }

            @Override
            public void setFieldSpeeds(Function<Pose2d, ChassisSpeeds> speeds) {
                periodicUpdater.apply(
                        odometryUpdater -> odometryUpdater.setModuleUpdater(
                                new SwerveOdometryUpdater.ChassisSpeedsUpdater(
                                        new SwerveOdometryUpdater.FieldOrienter(
                                                speeds,
                                                1.0 / periodicUpdater.getUpdateFrequency()
                                        )::apply,
                                        drivetrain,
                                        defaultApplyMode
                                )));
            }
        };
    }

    protected abstract SwerveModule.ApplyMode getDefaultApplyode();
    protected abstract SwerveModulePosition getPosition(SwerveModule module);
    protected abstract Gyro createGyro();
    protected abstract SwerveModule createModule(SwerveModuleConstants<?, ?, ?> moduleConstants);
    protected abstract PeriodicUpdater<SwerveOdometryUpdater> createPeriodicUpdater(SwerveOdometryUpdater odometryUpdater, Gyro gyro, SwerveModule[] modules);
}
