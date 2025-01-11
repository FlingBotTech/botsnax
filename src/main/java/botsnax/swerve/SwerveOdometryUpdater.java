package botsnax.swerve;

import botsnax.system.Gyro;
import botsnax.util.Updatable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Consumer;
import java.util.function.Function;

import static botsnax.swerve.SwerveModule.ApplyMode;
import static edu.wpi.first.math.kinematics.ChassisSpeeds.discretize;
import static edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds;

public class SwerveOdometryUpdater implements Updatable {
    public interface PoseUpdateListener {
        void onPoseUpdate(Pose2d pose);
    }

    public static class ChassisSpeedsUpdater implements PoseUpdateListener {
        private final Function<Pose2d, ChassisSpeeds> speedsFunction;
        private final GenericSwerveDrivetrain drivetrain;
        private final ApplyMode applyMode;

        public ChassisSpeedsUpdater(Function<Pose2d, ChassisSpeeds> speedsFunction, GenericSwerveDrivetrain drivetrain, ApplyMode applyMode) {
            this.speedsFunction = speedsFunction;
            this.drivetrain = drivetrain;
            this.applyMode = applyMode;
        }

        @Override
        public void onPoseUpdate(Pose2d pose) {
            drivetrain.apply(speedsFunction.apply(pose), applyMode);
        }
    }

    public static class FieldOrienter {
        private final Function<Pose2d, ChassisSpeeds> speedsFunction;
        private final double updatePeriod;

        public FieldOrienter(Function<Pose2d, ChassisSpeeds> speedsFunction, double updatePeriod) {
            this.speedsFunction = speedsFunction;
            this.updatePeriod = updatePeriod;
        }

        public ChassisSpeeds apply(Pose2d pose) {
            ChassisSpeeds fieldSpeeds = speedsFunction.apply(pose);
            return discretize(
                    fromFieldRelativeSpeeds(fieldSpeeds, pose.getRotation()),
                    updatePeriod);
        }
    }

    private final Gyro gyro;
    private final SwerveModule[] modules;
    private final Function<SwerveModule, SwerveModulePosition> positionGetter;
    private final Consumer<Pose2d> poseSetter;
    private final SwervePositionListener poseEstimator;
    private final PoseUpdateListener poseObserver;

    private final SwerveModulePosition[] positionBuffer;

    private PoseUpdateListener moduleUpdater = pose -> {};

    public SwerveOdometryUpdater(
            Gyro gyro,
            SwerveModule[] modules,
            Function<SwerveModule,SwerveModulePosition> positionGetter,
            Consumer<Pose2d> poseSetter,
            SwervePositionListener poseEstimator,
            PoseUpdateListener poseObserver) {
        this.gyro = gyro;
        this.modules = modules;
        this.positionGetter = positionGetter;
        this.poseSetter = poseSetter;
        this.poseEstimator = poseEstimator;
        this.poseObserver = poseObserver;

        this.positionBuffer = new SwerveModulePosition[modules.length];
    }

    public void update() {
        Rotation2d heading = gyro.getHeading();

        for (int i = 0; i < positionBuffer.length; i++) {
            positionBuffer[i] = positionGetter.apply(modules[i]);
        }

        Pose2d pose = poseEstimator.update(heading, positionBuffer);

        poseObserver.onPoseUpdate(pose);
        moduleUpdater.onPoseUpdate(pose);
    }

    public void setPose(Pose2d pose) {
        poseSetter.accept(pose);
    }

    public void setModuleUpdater(PoseUpdateListener poseUpdateListener) {
        this.moduleUpdater = poseUpdateListener;
    }
}
