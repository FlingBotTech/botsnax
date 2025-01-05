package botsnax.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import botsnax.system.Gyro;
import botsnax.util.Updatable;

import java.util.function.Function;

import static edu.wpi.first.math.kinematics.ChassisSpeeds.discretize;
import static edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds;
import static botsnax.swerve.SwerveModule.*;

public class SwerveOdometryUpdater implements Updatable {
    public interface UpdateListener {
        void onPoseUpdate(Pose2d pose);
    }

    public static class ChassisSpeedsUpdater implements UpdateListener {
        private final Function<Pose2d, ChassisSpeeds> speedsFunction;
        private final SwerveDrivetrain drivetrain;
        private final ApplyMode applyMode;

        public ChassisSpeedsUpdater(Function<Pose2d, ChassisSpeeds> speedsFunction, SwerveDrivetrain drivetrain, ApplyMode applyMode) {
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
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveModulePosition[] positionBuffer;

    private UpdateListener listener = pose -> {};

    public SwerveOdometryUpdater(
            Gyro gyro,
            SwerveModule[] modules,
            Function<SwerveModule,SwerveModulePosition> positionGetter,
            SwerveDrivePoseEstimator poseEstimator) {
        this.gyro = gyro;
        this.modules = modules;
        this.positionGetter = positionGetter;
        this.poseEstimator = poseEstimator;

        this.positionBuffer = new SwerveModulePosition[modules.length];
    }

    public void update() {
        Rotation2d heading = gyro.getHeading();

        for (int i = 0; i < positionBuffer.length; i++) {
            positionBuffer[i] = positionGetter.apply(modules[i]);
        }

        poseEstimator.update(heading, positionBuffer);

        listener.onPoseUpdate(poseEstimator.getEstimatedPosition());
    }

    public void setListener(UpdateListener updateListener) {
        this.listener = updateListener;
    }
}
