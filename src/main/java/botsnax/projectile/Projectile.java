package botsnax.projectile;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static java.lang.Double.NaN;
import static java.lang.Math.*;
import static java.lang.Math.max;

public class Projectile implements Sendable {
    private static final Measure<Velocity<Velocity<Distance>>> G = MetersPerSecondPerSecond.of(9.807);

    private Translation3d initialPosition;
    private Translation3d initialVelocity;

    public Projectile() {
        initialPosition = new Translation3d(0, 0, 0);
        initialVelocity = new Translation3d(0, 0, 0);
    }

    public Translation3d getInitialPosition() {
        return initialPosition;
    }

    public void setInitialPosition(Translation3d initialPosition) {
        this.initialPosition = initialPosition;
    }

    public Translation3d getInitialVelocity() {
        return initialVelocity;
    }

    public void setInitialVelocity(Translation3d initialVelocity) {
        this.initialVelocity = initialVelocity;
    }

    private static double getMinPositiveRoot(double a, double b, double c) {
        if (a != 0) {
            double rootOfDiscriminant = sqrt(pow(b, 2) - 4 * a * c);
            double root1 = (-b + rootOfDiscriminant) / (2 * a);
            double root2 = (-b - rootOfDiscriminant) / (2 * a);
            double minRoot = min(root1, root2);
            double maxRoot = max(root1, root2);

            if (minRoot > 0) {
                return minRoot;
            } else if (maxRoot > 0) {
                return maxRoot;
            } else {
                return NaN;
            }
        } else {
            double t = -c / b;

            if (t >= 0) {
                return t;
            } else {
                return NaN;
            }
        }
    }

    public Optional<Measure<Distance>> proximityToTargetIfAny(Vector<N3> target, Vector<N3> normal) {
        return timeOfIntersectionIfAny(target, normal).map(timeOfIntersection -> {
            Vector<N3> pointOfIntersection = atTime(timeOfIntersection);
            return BaseUnits.Distance.of(pointOfIntersection.minus(target).norm());
        });
    }

    public Optional<Measure<Distance>> proximityToTargetIfAny(Vector<N3> target) {
        return proximityToTargetIfAny(target, initialVelocity.toVector());
    }

    public Optional<Measure<Time>> timeOfIntersectionIfAny(Vector<N3> target, Vector<N3> normal) {
        double a = normal.dot(getSquareCoefficient());
        double b = normal.dot(initialVelocity.toVector());
        double c = normal.dot(initialPosition.toVector().minus(target));
        double t = getMinPositiveRoot(a, b, c);

        if (!Double.isNaN(t)) {
            return Optional.of(BaseUnits.Time.of(t));
        } else {
            return Optional.empty();
        }
    }

    public Vector<N3> atTime(Measure<Time> t) {
        return getSquareCoefficient().times(Math.pow(t.baseUnitMagnitude(), 2))
                .plus(initialVelocity.toVector().times(t.baseUnitMagnitude()))
                .plus(initialPosition.toVector());
    }

    private Optional<Measure<Time>> getFlightTime() {
        return timeOfIntersectionIfAny(new Vector<>(N3.instance), new Translation3d(0, 0, 1).toVector());
    }

    private Vector<N3> getSquareCoefficient() {
        return new Translation3d(0, 0, -0.5 * G.baseUnitMagnitude()).toVector();
    }

    private Optional<Measure<Distance>> getFlightDistance() {
        return getFlightTime().map(time -> {
            Vector<N3> collision = atTime(time);
            return BaseUnits.Distance.of(initialPosition.toVector().minus(collision).norm());
        });
    }

    private Translation3d convertRobotToAbsolute(Translation3d inRobotCoords, Pose2d robotPose) {
        return inRobotCoords
                .rotateBy(new Rotation3d(0, 0, robotPose.getRotation().getRadians()))
                .plus(new Translation3d(robotPose.getX(), robotPose.getY(), 0));
    }

    public static Measure<Angle> getBallisticAngle(Translation2d target, Measure<Velocity<Distance>> speed) {
        double a = (-G.baseUnitMagnitude() * target.getX()) / (2 * pow(speed.baseUnitMagnitude(), 2));
        double b = 1.0;
        double c = a - (target.getY() / target.getX());
        double angle = atan(getMinPositiveRoot(a, b, c));

        return Radians.of(angle);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("px", () -> initialPosition.getX(), null);
        builder.addDoubleProperty("py", () -> initialPosition.getY(), null);
        builder.addDoubleProperty("pz", () -> initialPosition.getZ(), null);
        builder.addDoubleProperty("vx", () -> initialVelocity.getX(), null);
        builder.addDoubleProperty("vy", () -> initialVelocity.getY(), null);
        builder.addDoubleProperty("vz", () -> initialVelocity.getZ(), null);
        builder.addDoubleProperty("Flight Time", () -> getFlightTime().map(t -> t.in(Seconds)).orElse(NaN), null);
        builder.addDoubleProperty("Flight Distance", () -> getFlightDistance().map(d -> d.in(Meters)).orElse(NaN), null);
    }
}
