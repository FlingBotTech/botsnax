package botsnax.swerve.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.swerve.GenericSwerveDrivetrain;

import java.util.function.Supplier;

import static botsnax.swerve.SwerveModule.ApplyMode;

public class ApplySpeedsCommand extends Command {
    private final GenericSwerveDrivetrain drivetrain;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final ApplyMode applyMode;

    public ApplySpeedsCommand(
            GenericSwerveDrivetrain drivetrain,
            Supplier<ChassisSpeeds> speedsSupplier,
            ApplyMode applyMode,
            Subsystem... requirements) {
        this.drivetrain = drivetrain;
        this.speedsSupplier = speedsSupplier;
        this.applyMode = applyMode;

        addRequirements(requirements);
    }

    @Override
    public void execute() {
        drivetrain.apply(speedsSupplier.get(), applyMode);
    }
}
