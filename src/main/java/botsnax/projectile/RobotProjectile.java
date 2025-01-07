package botsnax.projectile;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.units.Units.*;
import static java.lang.Double.NaN;

public class RobotProjectile implements Sendable {
    private final Projectile projectile = new Projectile();
    private final Vector<N3> target;

    public RobotProjectile(Vector<N3> target) {
        this.target = target;
    }

    private Translation3d convertRobotToAbsolute(Translation3d inRobotCoords, Pose2d robotPose) {
        return inRobotCoords
                .rotateBy(new Rotation3d(0, 0, robotPose.getRotation().getRadians()))
                .plus(new Translation3d(robotPose.getX(), robotPose.getY(), 0));
    }

    private Translation3d projectToGround(Translation3d t) {
        return new Translation3d(t.getX(), t.getY(), 0);
    }

    private Translation3d getReleaseVelocity(LinearVelocity projectileSpeed, Pose2d position, Translation3d releasePosition, Translation3d robotReleaseDirection, ChassisSpeeds velocity) {
        Translation3d shooterVelocity = robotReleaseDirection
                .rotateBy(new Rotation3d(0, 0, position.getRotation().getRadians()))
                .times(projectileSpeed.baseUnitMagnitude());

        Translation3d linearVelocity = new Translation3d(
                MetersPerSecond.of(velocity.vxMetersPerSecond).baseUnitMagnitude(),
                MetersPerSecond.of(velocity.vyMetersPerSecond).baseUnitMagnitude(),
                0);

        Translation3d groundReleasePosition = projectToGround(releasePosition)
                .minus(new Translation3d(position.getX(), position.getY(), 0));
        Translation3d rotationalVelocity = groundReleasePosition
                .rotateBy(new Rotation3d(0, 0, Degrees.of(90).in(Radians)))
                .times(velocity.omegaRadiansPerSecond);

        return shooterVelocity.plus(linearVelocity).plus(rotationalVelocity);
    }

    public void update(Pose2d pose, ChassisSpeeds velocity, Translation3d robotReleasePosition, Translation3d robotReleaseDirection, LinearVelocity shooterSpeed) {
        Translation3d releasePosition = convertRobotToAbsolute(robotReleasePosition, pose);
        projectile.setInitialPosition(releasePosition);
        projectile.setInitialVelocity(getReleaseVelocity(shooterSpeed, pose, releasePosition, robotReleaseDirection, velocity));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        projectile.initSendable(builder);
        builder.addDoubleProperty(
                "Projectile Target Distance",
                () -> projectile.proximityToTargetIfAny(target).map(d -> d.in(Inches)).orElse(NaN),
                null);
    }
}
