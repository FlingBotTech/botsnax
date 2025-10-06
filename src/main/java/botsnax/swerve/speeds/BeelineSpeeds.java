package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;

import static botsnax.swerve.speeds.CombineSpeeds.concatAndStop;
import static edu.wpi.first.units.Units.*;

public class BeelineSpeeds {
    private static final ChassisSpeeds ZERO_SPEED = new ChassisSpeeds();

    public interface LinearSpeedFunction {
        LinearVelocity getSpeed(Pose2d pose, Translation2d delta, double distance);

        static BeelineSpeeds.LinearSpeedFunction exponentialOf(Time stopTime, LinearVelocity maxSpeed, LinearVelocity minSpeed) {
            return (pose, delta, distance) -> {
                double maxSpeedMPS = maxSpeed.in(MetersPerSecond);
                double minSpeedMPS = minSpeed.in(MetersPerSecond);
                double stopSpeedMPS = distance / stopTime.in(Seconds);
                double speedMPS = Math.max(Math.min(maxSpeedMPS, stopSpeedMPS), minSpeedMPS);

                return MetersPerSecond.of(speedMPS);
            };
        }
    }

    public interface RotationFunction {
        AngularVelocity getRotation(Pose2d pose, Translation2d delta, double distance);

        static BeelineSpeeds.RotationFunction of(Distance nonRotationDistance, SpeedsFunction rotationSpeeds) {
            return (pose, delta, distance) -> distance > nonRotationDistance.in(Meters) ?
                    RadiansPerSecond.of(rotationSpeeds.apply(pose).omegaRadiansPerSecond) :
                    RadiansPerSecond.zero();
        }
    }

    public interface StopFunction {
        boolean isStopped(Pose2d pose, LinearVelocity requestedVelocity, Translation2d delta, double distance);

        static BeelineSpeeds.StopFunction of(LinearVelocity maxStopSpeed, Distance threshold, State state) {
            return (pose, requestedVelocity, delta, distance) -> {
                state.linearStopThreshold = threshold;
                return (requestedVelocity.baseUnitMagnitude() < maxStopSpeed.baseUnitMagnitude()) &&
                        (distance < threshold.in(Meters));
            };
        }
    }

    public static class State {
        private double distance;
        private LinearVelocity linearVelocity;
        private AngularVelocity angularVelocity;
        private Distance linearStopThreshold;
        private boolean stopped;

        public double getDistance() {
            return distance;
        }

        public LinearVelocity getLinearVelocity() {
            return linearVelocity;
        }

        public AngularVelocity getAngularVelocity() {
            return angularVelocity;
        }

        public Distance getLinearStopThreshold() {
            return linearStopThreshold;
        }

        public boolean isStopped() {
            return stopped;
        }
    }

    public static class DefaultLogger implements Consumer<State> {
        private final DoublePublisher distanceP;
        private final DoublePublisher speedP;
        private final DoublePublisher angularSpeedP;
        private final DoublePublisher linearStopThresholdP;
        private final BooleanPublisher stoppedP;

        public DefaultLogger() {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("Beeline");
            distanceP = table.getDoubleTopic("distance").publish();
            speedP = table.getDoubleTopic("speed").publish();
            angularSpeedP = table.getDoubleTopic("angularSpeedRPS").publish();
            linearStopThresholdP = table.getDoubleTopic("linearStopThreshold").publish();
            stoppedP = table.getBooleanTopic("stopped").publish();
        }

        @Override
        public void accept(State state) {
            distanceP.accept(state.getDistance());
            speedP.accept(state.getLinearVelocity().in(MetersPerSecond));
            angularSpeedP.accept(state.getAngularVelocity().in(RotationsPerSecond));
            linearStopThresholdP.accept(state.getLinearStopThreshold().in(Meters));
            stoppedP.accept(state.isStopped());
        }
    }

    public static SpeedsFunction of(Pose2d targetPose, LinearSpeedFunction speedFunction, RotationFunction rotationFunction, StopFunction stopFunction, State state, Consumer<State> logger) {
        return pose -> {
            Translation2d delta = targetPose.getTranslation().minus(pose.getTranslation());
            double distance = delta.getNorm();
            LinearVelocity speed = speedFunction.getSpeed(pose, delta, distance);
            AngularVelocity rotation = rotationFunction.getRotation(pose, delta, distance);
            boolean stop = stopFunction.isStopped(pose, speed, delta, distance);
            Translation2d velocity = delta.times(speed.in(MetersPerSecond) / distance);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), rotation.in(RadiansPerSecond));

            state.distance = distance;
            state.stopped = stop;
            state.linearVelocity = speed;
            state.angularVelocity = rotation;

            if (logger != null) {
                logger.accept(state);
            }

            return stop ? ZERO_SPEED : chassisSpeeds;
        };
    }

    public static SpeedsFunction of(Pose2d targetPose, Time stopTime, Time angularStopTime, LinearVelocity maxSpeed, ThresholdParams thresholdParams, Consumer<State> logger, Consumer<FacingSpeeds.State> facingLogger, Subsystem... requirements) {
        FacingSpeeds.AngleFunction angleFunction = pose -> targetPose.getRotation();

        return of(targetPose, angleFunction, stopTime, angularStopTime, maxSpeed, thresholdParams, logger, facingLogger);
    }

    public static SpeedsFunction of(Pose2d targetPose, Translation2d facingToward, Time stopTime, Time angularStopTime, LinearVelocity maxSpeed, ThresholdParams thresholdParams, Consumer<State> logger, Consumer<FacingSpeeds.State> facingLogger) {
        return of(targetPose, FacingSpeeds.AngleFunction.of(facingToward, targetPose.getRotation()), stopTime, angularStopTime, maxSpeed, thresholdParams, logger, facingLogger);
    }

    public static SpeedsFunction of(Pose2d targetPose, FacingSpeeds.AngleFunction angleFunction, Time stopTime, Time angularStopTime, LinearVelocity maxSpeed, ThresholdParams thresholdParams, Consumer<State> logger, Consumer<FacingSpeeds.State> facingLogger) {
        AngularVelocity maxAngularVelocity = thresholdParams.getAngularVelocity(maxSpeed).times(0.7);
        State state = new State();

        SpeedsFunction facingSpeeds = FacingSpeeds.of(
                angleFunction,
                angularStopTime,
                maxAngularVelocity,
                thresholdParams.minAngularVelocity(),
                facingLogger
        );

        SpeedsFunction postFacingSpeeds = FacingSpeeds.of(
                pose -> targetPose.getRotation(),
                angularStopTime,
                maxAngularVelocity,
                thresholdParams,
                facingLogger
        );

        Distance threshold = thresholdParams.getThreshold();
        BeelineSpeeds.LinearSpeedFunction speedFunction = LinearSpeedFunction.exponentialOf(stopTime, maxSpeed, thresholdParams.minSpeed());
        BeelineSpeeds.RotationFunction thresholdRotationFunction = BeelineSpeeds.RotationFunction.of(threshold.times(5), facingSpeeds);
        BeelineSpeeds.StopFunction thresholdStopFunction = BeelineSpeeds.StopFunction.of(maxSpeed.times(0.05), threshold, state);

        return concatAndStop(
                BeelineSpeeds.of(
                        targetPose,
                        speedFunction,
                        thresholdRotationFunction,
                        thresholdStopFunction,
                        state,
                        logger
                ),
                postFacingSpeeds);
    }
}