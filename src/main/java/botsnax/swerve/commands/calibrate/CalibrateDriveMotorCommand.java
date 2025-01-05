package botsnax.swerve.commands.calibrate;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.commands.calibrate.AwaitStableValueCommand;
import botsnax.commands.calibrate.VelocityCalibration;
import botsnax.commands.calibrate.VelocityCalibrationData;
import botsnax.swerve.SwerveModule;
import botsnax.system.motor.AngleSetter;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class CalibrateDriveMotorCommand extends SequentialCommandGroup {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final int moduleIndex;
    private final AngleSetter angleSetter;
    private final VelocityCalibrationData data = new VelocityCalibrationData();

    public CalibrateDriveMotorCommand(
            SwerveDriveKinematics kinematics,
            SwerveModule[] modules,
            int moduleIndex,
            AngleSetter angleSetter,
            Consumer<VelocityCalibration> consumer,
            Subsystem ... requirements) {
        this.kinematics = kinematics;
        this.modules = modules;
        this.moduleIndex = moduleIndex;
        this.angleSetter = angleSetter;

        addCommands(park(requirements));

        double[] percentages = new double[] { 0.15, 0.3, 0.45, /* 0.6 */ };

        for (double percentage : percentages) {
            addCommands(runAtSpeed(percentage * 12.0, requirements));
        }

        addCommands(park(requirements));
        addCommands(new InstantCommand(() -> consumer.accept(data.solve()), requirements));
        addCommands(waitForPark());
    }

    private Command runAtSpeed(double voltage, Subsystem... requirements) {
        SwerveModule module = modules[moduleIndex];

        return new RotateAtVoltageCommand(kinematics, modules, angleSetter, moduleIndex, voltage, requirements)
                .andThen(new AwaitStableValueCommand(
                        Seconds.of(2),
                        RadiansPerSecond.of(0.01).baseUnitMagnitude(),
                        () -> module.getDrive().getVelocity().in(RadiansPerSecond),
                        result -> data.add(RadiansPerSecond.of(result), voltage)
                ));
    }

    private Command park(Subsystem ... requirements) {
        return runAtSpeed(0, requirements);
    }

    private Command waitForPark(Subsystem... requirements) {
        SwerveModule module = modules[moduleIndex];

        return new AwaitStableValueCommand(
                Seconds.of(2),
                RadiansPerSecond.of(0.01).baseUnitMagnitude(),
                () -> module.getDrive().getVelocity().in(RadiansPerSecond),
                value -> {},
                requirements
        );
    }
}
