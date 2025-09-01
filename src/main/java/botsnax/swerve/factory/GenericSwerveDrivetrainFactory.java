package botsnax.swerve.factory;

import botsnax.swerve.*;
import botsnax.swerve.listener.PoseLogger;
import botsnax.swerve.sim.GenericSwerveSim;
import botsnax.swerve.sim.IdealizedSwerveDrivetrain;
import botsnax.swerve.sim.IdealizedSwerveSim;
import botsnax.system.Gyro;
import botsnax.util.PeriodicUpdater;
import botsnax.util.SimRunner;
import botsnax.vision.PoseEstimate;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Mass;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import static edu.wpi.first.units.Units.*;
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

        List<SwerveModule> moduleList = new ArrayList<>(moduleConstants.length);
        for (int moduleId = 0; moduleId < moduleConstants.length; moduleId++) {
            moduleList.add(createModule(moduleId, moduleConstants[moduleId]));
        }
        SwerveModule[] modules = moduleList.toArray(new SwerveModule[0]);

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

        DoublePublisher poseEstimateErrorPublisher = NetworkTableInstance.getDefault().getDoubleTopic("PoseEstimateError").publish();

        SwerveOdometryUpdater odometryUpdater = new SwerveOdometryUpdater(
                gyro,
                modules,
                this::getPosition,
                poseEstimator::resetPose,
                (heading, modulePositions) -> {
                    poseEstimator.update(heading, modulePositions);

                    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();

                    if (isSimulation()) {
                        Pose2d actualPose = idealizedSwerveSim.getPose();
                        double error = actualPose.getTranslation().minus(estimatedPose.getTranslation()).getNorm();

                        poseEstimateErrorPublisher.set(error);

                        return actualPose;
                    } else {
                        return estimatedPose;
                    }
                },
                measurement -> poseEstimator.addVisionMeasurement(measurement.pose(), measurement.timestamp().in(Seconds), measurement.stdDevs()),
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
        SwerveModule.ApplyMode defaultApplyMode = getDefaultApplyMode();

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
            public Pose2d getSimPose() {
                return idealizedSwerveSim.getPose();
            }

            @Override
            public void setPose(Pose2d pose) {
                periodicUpdater.apply(odometryUpdater -> odometryUpdater.setPose(pose));
            }

            @Override
            public void setSimPose(Pose2d pose) {
                idealizedSwerveSim.setPose(pose);
            }

            @Override
            public void setRobotSpeeds(Function<Pose2d, ChassisSpeeds> speeds) {
                periodicUpdater.apply(
                        odometryUpdater -> odometryUpdater.setModuleUpdater(
                                new SwerveOdometryUpdater.ChassisSpeedsUpdater(
                                        speeds,
                                        drivetrain,
                                        defaultApplyMode
                                )));
            }

            @Override
            public void setFieldSpeeds(Function<Pose2d, ChassisSpeeds> speeds) {
                setRobotSpeeds(
                        new SwerveOdometryUpdater.FieldOrienter(
                                speeds,
                                1.0 / periodicUpdater.getUpdateFrequency()
                        )::apply
                );
            }

            @Override
            public void addVisionMeasurement(PoseEstimate estimate) {
                periodicUpdater.apply(updater -> updater.addVisionMeasurement(estimate));
            }
        };
    }

    protected abstract SwerveModule.ApplyMode getDefaultApplyMode();
    protected abstract SwerveModulePosition getPosition(SwerveModule module);
    protected abstract Gyro createGyro();
    protected abstract SwerveModule createModule(int moduleId, SwerveModuleConstants<?, ?, ?> moduleConstants);
    protected abstract PeriodicUpdater<SwerveOdometryUpdater> createPeriodicUpdater(SwerveOdometryUpdater odometryUpdater, Gyro gyro, SwerveModule[] modules);
}
