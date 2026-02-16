package botsnax.swerve.phoenix;

import botsnax.swerve.GenericSwerveDrivetrain;
import botsnax.swerve.SwerveController;
import botsnax.swerve.sim.IdealizedSwerveSim;
import botsnax.vision.PoseEstimate;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class PhoenixSwerveController implements SwerveController, Subsystem {
    private final SwerveDrivetrain<?, ?, ?> phoenixDrivetrain;
    private final GenericSwerveDrivetrain genericDrivetrain;
    private final IdealizedSwerveSim sim;
    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;
    private final SwerveModule.DriveRequestType driveRequestType;

    public PhoenixSwerveController(
            SwerveDrivetrain<?, ?, ?> phoenixDrivetrain,
            GenericSwerveDrivetrain genericDrivetrain,
            IdealizedSwerveSim sim,
            SwerveModule.DriveRequestType driveRequestType) {
        this.phoenixDrivetrain = phoenixDrivetrain;
        this.genericDrivetrain = genericDrivetrain;
        this.sim = sim;
        this.driveRequestType = driveRequestType;

        this.applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
                .withDriveRequestType(driveRequestType);
        this.applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
                .withDriveRequestType(driveRequestType);
    }

    @Override
    public GenericSwerveDrivetrain getDrivetrain() {
        return genericDrivetrain;
    }

    public SwerveDrivetrain<?, ?, ?> getPhoenixDrivetrain() {
        return phoenixDrivetrain;
    }

    @Override
    public Pose2d getPose() {
        return phoenixDrivetrain.getState().Pose;
    }

    @Override
    public Pose2d getSimPose() {
        return sim.getPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        phoenixDrivetrain.resetPose(pose);
    }

    @Override
    public void setSimPose(Pose2d pose) {
        sim.setPose(pose);
    }

    private Supplier<Pose2d> getPoseSupplier() {
        return isSimulation() ? this::getSimPose : this::getPose;
    }

    private void setSwerveRequestFunction(Function<Pose2d, SwerveRequest> requestFunction) {
        Command defaultCommand = getDefaultCommand();
        if (defaultCommand != null) {
            defaultCommand.cancel();
        }

        Supplier<Pose2d> poseSupplier = getPoseSupplier();

        setDefaultCommand(run(() -> {
            phoenixDrivetrain.setControl(requestFunction.apply(poseSupplier.get()));
        }));
    }

    @Override
    public void setRobotSpeeds(Function<Pose2d, ChassisSpeeds> speeds) {
        setSwerveRequestFunction(pose -> applyRobotSpeeds.withSpeeds(speeds.apply(pose)));
    }

    @Override
    public void setFieldSpeeds(Function<Pose2d, ChassisSpeeds> speeds) {
        setSwerveRequestFunction(pose -> applyFieldSpeeds.withSpeeds(speeds.apply(pose)));
    }

    @Override
    public void addVisionMeasurement(PoseEstimate estimate) {
        phoenixDrivetrain.addVisionMeasurement(
                estimate.pose(),
                Utils.fpgaToCurrentTime(estimate.timestamp().in(Seconds)),
                estimate.stdDevs());
    }
}
