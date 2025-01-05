package botsnax.swerve;

import botsnax.swerve.commands.calibrate.CalibrateDriveMotorsCommand;
import botsnax.swerve.commands.calibrate.StoreEncoderOffsetsCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.system.motor.AngleSetter;
import botsnax.util.PeriodicUpdater;

import java.util.function.Function;

public class SwerveDrivetrain {
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final PeriodicUpdater<SwerveOdometryUpdater> periodicUpdater;

    public SwerveDrivetrain(SwerveDriveKinematics kinematics, SwerveModule[] modules, PeriodicUpdater<SwerveOdometryUpdater> periodicUpdater) {
        this.kinematics = kinematics;
        this.modules = modules;
        this.periodicUpdater = periodicUpdater;
    }

    public double getUpdateFrequency() {
        return periodicUpdater.getUpdateFrequency();
    }

    public void apply(ChassisSpeeds speeds, SwerveModule.ApplyMode mode) {
        apply(kinematics.toSwerveModuleStates(speeds), mode);
    }

    public void apply(SwerveModuleState[] states, SwerveModule.ApplyMode mode) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].apply(states[i], mode);
        }
    }

    public void withSpeedsFunction(Function<Pose2d, ChassisSpeeds> speedsFunction, SwerveModule.ApplyMode applyMode) {
        periodicUpdater.apply(
                odometryUpdater -> odometryUpdater.setListener(
                        new SwerveOdometryUpdater.ChassisSpeedsUpdater(speedsFunction, this, applyMode)));
    }

    public void stop() {
        periodicUpdater.apply(
                odometryUpdater -> odometryUpdater.setListener(pose -> {})
        );
    }

    public Command calibrateDriveMotors(AngleSetter angleSetter, Subsystem... requirements) {
        return new CalibrateDriveMotorsCommand(kinematics, modules, angleSetter, requirements);
    }

    public Command storeEncoderOffsets(Subsystem ... requirements) {
        return new StoreEncoderOffsetsCommand(modules, requirements);
    }
}
