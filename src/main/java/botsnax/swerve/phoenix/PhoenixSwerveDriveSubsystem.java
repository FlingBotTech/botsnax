package botsnax.swerve.phoenix;

import botsnax.swerve.sim.SwerveSim;
import botsnax.vision.PoseEstimate;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import static botsnax.swerve.phoenix.PhoenixSwerveDrivetrainBuilder.getEncoderOffsetPreferenceName;
import static botsnax.util.phoenix.CANcoderUtil.saveOffsetToZeroCurrentPosition;
import static edu.wpi.first.units.Units.*;
import static java.lang.Double.NaN;

public class PhoenixSwerveDriveSubsystem extends SubsystemBase {
    private final PhoenixSwerveDrivetrainBuilder builder;
    protected final Optional<PhoenixSwerveDrivetrain> drivetrainIfAny;
    protected final Optional<SwerveSim> simIfAny;

    private final StructPublisher<Pose2d> odometryPublisher;
    private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
    private final StructArrayPublisher<SwerveModuleState> moduleTargetPublisher;

    public PhoenixSwerveDriveSubsystem(PhoenixSwerveDrivetrainBuilder builder, Function<PhoenixSwerveDrivetrain, SwerveSim> startSim) {
        this.builder = builder;

        drivetrainIfAny = builder.createDrivetrain();
        simIfAny = builder.startSim(startSim);

        odometryPublisher = getPosePublisher("Odometry");
        moduleStatePublisher = getModulePublisher("SwerveModuleStates");
        moduleTargetPublisher = getModulePublisher("SwerveModuleTargets");

        drivetrainIfAny.ifPresent(drivetrain -> {
            drivetrain.registerTelemetry(swerveDriveState -> {
                odometryPublisher.set(swerveDriveState.Pose);
                moduleStatePublisher.set(swerveDriveState.ModuleStates);
                moduleTargetPublisher.set(swerveDriveState.ModuleTargets);
            });
        });

        SendableRegistry.add(this, "DriveSubsystem");
        SmartDashboard.putData(this);
    }

    private StructArrayPublisher<SwerveModuleState> getModulePublisher(String name) {
        return NetworkTableInstance
                .getDefault()
                .getStructArrayTopic(name, SwerveModuleState.struct)
                .publish();
    }

    private StructPublisher<Pose2d> getPosePublisher(String name) {
        return NetworkTableInstance
                .getDefault()
                .getStructTopic(name, Pose2d.struct)
                .publish();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty( "x", () -> getOdometryPose() != null ? getOdometryPose().getX() : NaN, null);
        builder.addDoubleProperty( "y", () -> getOdometryPose() != null ? getOdometryPose().getY() : NaN, null);
        builder.addDoubleProperty("yaw", () -> getOdometryPose() != null ? getOdometryPose().getRotation().getDegrees() : NaN, null);
    }

    private Pose2d getOdometryPose() {
        return drivetrainIfAny.map(drivetrain -> drivetrain.getState().Pose).orElse(null);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return drivetrainIfAny
                .map(drivetrain -> run(() -> drivetrain.setControl(requestSupplier.get())))
                .orElse(runOnce(() -> System.err.println("Drive subsystem not available")));
    }

    public Command applyRequest(SwerveRequest request) {
        return drivetrainIfAny
                .map(drivetrain -> runOnce(() -> drivetrain.setControl(request)))
                .orElse(runOnce(() -> System.err.println("Drive subsystem not available")));
    }

//    public Command calibrateDriveMotors() {
//        return drivetrainIfAny
//                .map(drivetrain -> (Command) new CalibrateDriveMotorsCommand(drivetrain, builder.getModuleConstants(), this))
//                .orElse(runOnce(() -> System.err.println("Drive subsystem not available")));
//    }

    public Command storeEncoderOffsets() {
        Command[] commands = Arrays.stream(builder.getModuleConstants()).map(module -> {
            try (CANcoder cancoder = new CANcoder(module.EncoderId, builder.getDrivetrainConstants().CANBusName)) {
                return runOnce(() -> saveOffsetToZeroCurrentPosition(getEncoderOffsetPreferenceName(module.EncoderId), cancoder));
            }
        }).toArray(Command[]::new);

        return new SequentialCommandGroup(commands);
    }

//    public SwerveModule getModule(int index) {
//        return drivetrainIfAny.map(drivetrain -> drivetrain.getModule(index))
//                .orElseThrow();
//    }

    public Command getContinuousDriveCommand(
            SwerveRequest.RobotCentric drive,
            Supplier<LinearVelocity> vxSupplier,
            Supplier<LinearVelocity> vySupplier,
            Supplier<AngularVelocity> vrSupplier) {
        return applyRequest(() -> drive
                .withVelocityX(vxSupplier.get().in(MetersPerSecond))
                .withVelocityY(vySupplier.get().in(MetersPerSecond))
                .withRotationalRate(vrSupplier.get().in(RadiansPerSecond)));
    }

    public void addVisionMeasurement(Optional<PoseEstimate> measurementIfAny) {
        drivetrainIfAny.ifPresent(drivetrain -> {
            measurementIfAny.ifPresent(measurement -> {
                drivetrain.addVisionMeasurement(
                        measurement.pose(),
                        measurement.timestamp().in(Seconds),
                        measurement.stdDevs());
            });
        });
    }
}
