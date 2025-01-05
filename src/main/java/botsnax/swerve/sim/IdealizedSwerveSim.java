package botsnax.swerve.sim;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.math.VecBuilder.fill;
import static edu.wpi.first.units.Units.*;

public class IdealizedSwerveSim implements SwerveSim {
    private final SwerveDrivetrain drivetrain;

    private ChassisSpeeds velocity;
    private final Field2d field;

    public IdealizedSwerveSim(SwerveDrivetrain drivetrain, Pose2d initialPose) {
        this.drivetrain = drivetrain;
        this.field = new Field2d();

        velocity = new ChassisSpeeds();

        field.setRobotPose(initialPose);
        SmartDashboard.putData(field);
    }

    public Pose2d getPose() {
        return field.getRobotPose();
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public void update(ChassisSpeeds requestedVelocity, Measure<Time> dt) {
        Vector<N2> linearVelocity = fill(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        Vector<N2> requestedLinearVelocity = fill(requestedVelocity.vxMetersPerSecond, requestedVelocity.vyMetersPerSecond);

        Measure<Velocity<Distance>> speed = MetersPerSecond.of(linearVelocity.norm());
        Measure<Velocity<Distance>> requestedSpeed = MetersPerSecond.of(requestedLinearVelocity.norm());

        Vector<N2> direction = (requestedSpeed.baseUnitMagnitude() != 0) ? requestedLinearVelocity.div(requestedSpeed.in(MetersPerSecond)) :
                (speed.baseUnitMagnitude() != 0) ? linearVelocity.div(speed.in(MetersPerSecond)) : fill(0, 0);

        Measure<Velocity<Distance>> updatedSpeed = speed.plus(drivetrain.getLinearDrive().getVelocityChange(requestedSpeed, speed, dt));
        Vector<N2> updatedLinearVelocity = direction.times(updatedSpeed.in(MetersPerSecond));

        Measure<Distance> dx = drivetrain.getLinearDrive().getDistanceChange(requestedSpeed, speed, dt);
        Vector<N2> position = fill(field.getRobotPose().getX(), field.getRobotPose().getY());
        Vector<N2> updatedPosition = direction.times(dx.in(Meters)).plus(position);

        Measure<Angle> angle = Radians.of(field.getRobotPose().getRotation().getRadians());
        Measure<Velocity<Angle>> requestedOmega = RadiansPerSecond.of(requestedVelocity.omegaRadiansPerSecond);
        Measure<Velocity<Angle>> omega = RadiansPerSecond.of(velocity.omegaRadiansPerSecond);

        Measure<Angle> updatedAngle = angle.plus(drivetrain.getAngularDrive().getAngleChange(requestedOmega, omega, dt));
        Measure<Velocity<Angle>> updatedAngularVelocity = omega.plus(drivetrain.getAngularDrive().getVelocityChange(requestedOmega, omega, dt));

        velocity = new ChassisSpeeds(updatedLinearVelocity.get(0), updatedLinearVelocity.get(1), updatedAngularVelocity.in(RadiansPerSecond));
        Pose2d pose = new Pose2d(updatedPosition.get(0), updatedPosition.get(1), Rotation2d.fromRadians(updatedAngle.in(Radians)));
        field.setRobotPose(pose);
    }
}
