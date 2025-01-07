package botsnax.swerve;

import botsnax.swerve.sim.IdealizedSwerveSim;
import botsnax.swerve.sim.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static java.lang.Math.*;

public class IdealizedDriveSubsystem extends SubsystemBase {
    private final SwerveDrivetrain drivetrain;
    private final ChassisSpeeds requestedVelocity;
    protected final IdealizedSwerveSim sim;

    public IdealizedDriveSubsystem(SwerveDrivetrain drivetrain, Pose2d initialPose) {
        this.drivetrain = drivetrain;

        sim = drivetrain.createSim(initialPose);
        requestedVelocity = new ChassisSpeeds();

        SendableRegistry.add(this, "Drive");
        SmartDashboard.putData(this);
    }

    protected void setRequestedVelocity(ChassisSpeeds requestedVelocity) {
        this.requestedVelocity.vxMetersPerSecond = requestedVelocity.vxMetersPerSecond;
        this.requestedVelocity.vyMetersPerSecond = requestedVelocity.vyMetersPerSecond;
        this.requestedVelocity.omegaRadiansPerSecond = requestedVelocity.omegaRadiansPerSecond;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Requested Velocity X", () -> requestedVelocity.vxMetersPerSecond, value -> requestedVelocity.vxMetersPerSecond = value);
        builder.addDoubleProperty("Requested Velocity Y", () -> requestedVelocity.vyMetersPerSecond, value -> requestedVelocity.vyMetersPerSecond = value);
        builder.addDoubleProperty("Requested Velocity R", () -> requestedVelocity.omegaRadiansPerSecond, value -> requestedVelocity.omegaRadiansPerSecond = value);
    }

    @Override
    public void simulationPeriodic() {
        sim.update(requestedVelocity, Milliseconds.of(20));
    }

    public Pose2d getPose() {
        return isSimulation() ? sim.getPose() : new Pose2d(0, 0, new Rotation2d(0));
    }

    public ChassisSpeeds getVelocity() {
        return isSimulation() ? sim.getVelocity() : new ChassisSpeeds(0, 0, 0);
    }

    public Command getJoystickControlCommand(CommandPS4Controller joystick) {
        return run(() -> {
            final double deadband = 0.1;
            final AngularVelocity maxOmega = RadiansPerSecond.of(2.5 * PI);

            double lx = joystick.getLeftX();
            double ly = joystick.getLeftY();
            double rx = joystick.getRightX();

            double v = sqrt(pow(lx, 2) + pow(ly, 2));
            double vx = lx * drivetrain.getMaxLinearSpeed().in(MetersPerSecond) / v;
            double vy = -ly * drivetrain.getMaxLinearSpeed().in(MetersPerSecond) / v;
            double vr = maxOmega.times(rx).magnitude();

            if (v > deadband) {
                requestedVelocity.vxMetersPerSecond = vx;
                requestedVelocity.vyMetersPerSecond = vy;
            } else {
                requestedVelocity.vxMetersPerSecond = 0;
                requestedVelocity.vyMetersPerSecond = 0;
            }

            if (abs(rx) > deadband) {
                requestedVelocity.omegaRadiansPerSecond = vr;
            } else {
                requestedVelocity.omegaRadiansPerSecond = 0;
            }
        });
    }

    public Command calibrateDriveMotors() {
        return runOnce(() -> {});
    }
}
