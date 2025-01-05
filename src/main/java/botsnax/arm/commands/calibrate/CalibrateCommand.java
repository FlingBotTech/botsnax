package botsnax.arm.commands.calibrate;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.arm.commands.AwaitStabilityCommand;

import java.util.function.Consumer;

import static edu.wpi.first.units.ImmutableMeasure.ofRelativeUnits;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;

public class CalibrateCommand extends SequentialCommandGroup {
    private ArmCalibration calibration;

    public CalibrateCommand(ArmCalibrationParams arm, double maxVoltage, Consumer<ArmCalibration> consumer, Subsystem ... requirements) {
        addCommands(
                new CalibrateGearRatioCommand(
                        arm,
                        maxVoltage,
                        calibration -> this.calibration = calibration),
                new CalibrateGravityCommand(
                        arm,
                        Degrees.of(5),
                        () -> calibration,
                        calibration -> this.calibration = calibration),
                new CalibrateVelocityCommand(
                        arm,
                        maxVoltage,
                        () -> calibration,
                        calibration -> this.calibration = calibration
                ),
                park(arm),
                new InstantCommand(() -> consumer.accept(calibration), requirements)
        );
    }

    private Command park(ArmCalibrationParams arm) {
        return new AwaitStabilityCommand(
                arm,
                () -> calibration.createController(
                        state -> ofRelativeUnits(1, Degrees),
                        ofRelativeUnits(1, Second),
                        ofRelativeUnits(25, Degrees.per(Second))),
                result -> {}
        )
                .andThen(new InstantCommand(() -> arm.setVoltage(0), arm.requirements()));
    }
}
