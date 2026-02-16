package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import java.util.function.Consumer;

import static botsnax.swerve.speeds.FacingSpeeds.AngularSpeedFunction.exponentialOf;
import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;
import static java.lang.Math.*;

public class FacingSpeeds {
    private static final ChassisSpeeds ZERO_SPEED = new ChassisSpeeds();

    public interface AngleFunction {
        Rotation2d getAngle(Pose2d pose);

        static FacingSpeeds.AngleFunction of(Translation2d facingToward) {
            return pose -> {
                Translation2d delta = facingToward.minus(pose.getTranslation());
                return new Rotation2d(delta.getX(), delta.getY());
            };
        }

        static FacingSpeeds.AngleFunction of(Translation2d facingToward, Rotation2d angle) {
            return pose -> {
                Translation2d delta = facingToward.minus(pose.getTranslation());
                Rotation2d deltaRotation = new Rotation2d(delta.getX(), delta.getY());

                return new Rotation2d(deltaRotation.getCos() + angle.getCos(), deltaRotation.getSin() + angle.getSin());
            };
        }
    }

    public interface AngularSpeedFunction {
        double getSpeedRadPS(double angleRad);

        static FacingSpeeds.AngularSpeedFunction exponentialOf(Time stopTime, AngularVelocity maxVelocity, AngularVelocity minVelocity) {
            return angleRad -> {
                double reachVelocity = angleRad / stopTime.in(Seconds);
                double velocity = max(min(maxVelocity.in(RadiansPerSecond), abs(reachVelocity)), minVelocity.in(RadiansPerSecond)) * signum(reachVelocity);

                return velocity;
            };
        }
    }

    public interface StopFunction {
        boolean isStopped(Pose2d pose, double velocityRadPS, double deltaRad, State state);

        static FacingSpeeds.StopFunction of(AngularVelocity maxStopSpeed, Angle threshold) {
            return (pose, velocityRadPS, deltaRad, state) -> {
                state.thresholdDeg = threshold.in(Degrees);
                return (velocityRadPS < maxStopSpeed.in(RadiansPerSecond)) &&
                        (abs(deltaRad) < threshold.in(Radians));
            };
        }
    }

    public static class State {
        private double thresholdDeg;
        private double distanceDeg;
        private double speedRPS;
        private boolean stopped;

        public double getThresholdDeg() {
            return thresholdDeg;
        }

        public double getDistanceDeg() {
            return distanceDeg;
        }

        public double getSpeedRPS() {
            return speedRPS;
        }

        public boolean isStopped() {
            return stopped;
        }
    }

    public static class DefaultLogger implements Consumer<State> {
        private final DoublePublisher thresholdPublisher;
        private final DoublePublisher distancePublisher;
        private final DoublePublisher speedPublisher;
        private final BooleanPublisher stoppedPublisher;

        public DefaultLogger() {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("Facing");
            thresholdPublisher = table.getDoubleTopic("thresholdD").publish();
            distancePublisher = table.getDoubleTopic("distanceD").publish();
            speedPublisher = table.getDoubleTopic("speedRPS").publish();
            stoppedPublisher = table.getBooleanTopic("stopped").publish();
        }

        @Override
        public void accept(State state) {
            thresholdPublisher.accept(state.getThresholdDeg());
            distancePublisher.accept(state.getDistanceDeg());
            speedPublisher.accept(state.getSpeedRPS());
            stoppedPublisher.accept(state.isStopped());
        }
    }

    public static SpeedsFunction of(AngleFunction angleFunction, AngularSpeedFunction speedFunction, StopFunction stopFunction, State state, Consumer<State> logger) {
        return pose -> {
            Rotation2d angle = angleFunction.getAngle(pose);
            double delta = angleModulus(angle.getRadians() - pose.getRotation().getRadians());
            double velocityRadPS = speedFunction.getSpeedRadPS(delta);
            boolean stop = stopFunction.isStopped(pose, velocityRadPS, delta, state);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, velocityRadPS);

            state.distanceDeg = abs(delta * 180) / Math.PI;
            state.speedRPS = velocityRadPS / (2 * Math.PI);
            state.stopped = stop;

            if (logger != null) {
                logger.accept(state);
            }

            return stop ? ZERO_SPEED : chassisSpeeds;
        };
    }

    public static SpeedsFunction of(FacingSpeeds.AngleFunction angleFunction, Time stopTime, AngularVelocity maxSpeed, AngularVelocity minSpeed, Consumer<State> logger) {
        State state = new State();
        FacingSpeeds.AngularSpeedFunction speedFunction = exponentialOf(stopTime, maxSpeed, minSpeed);

        state.thresholdDeg = Double.POSITIVE_INFINITY;

        return FacingSpeeds.of(
                angleFunction,
                speedFunction,
                (pose, velocityRadPS, deltaRad, stateValue) -> false,
                state,
                logger
        );
    }

    public static SpeedsFunction of(FacingSpeeds.AngleFunction angleFunction, Time stopTime, AngularVelocity maxSpeed, ThresholdParams thresholdParams, Consumer<State> logger) {
        State state = new State();
        Angle threshold = thresholdParams.getAngularThreshold();
        FacingSpeeds.AngularSpeedFunction speedFunction = exponentialOf(stopTime, maxSpeed, thresholdParams.minAngularVelocity());
        FacingSpeeds.StopFunction thresholdStopFunction = FacingSpeeds.StopFunction.of(maxSpeed.times(0.05), threshold);

        state.thresholdDeg = threshold.in(Degrees);

        return FacingSpeeds.of(
                angleFunction,
                speedFunction,
                thresholdStopFunction,
                state,
                logger
        );
    }
}
