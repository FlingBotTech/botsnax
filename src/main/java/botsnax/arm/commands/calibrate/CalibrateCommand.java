package botsnax.arm.commands.calibrate;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.arm.commands.AwaitStabilityCommand;

import java.util.function.Consumer;

import static edu.wpi.first.units.ImmutableMeasure.ofRelativeUnits;
import static edu.wpi.first.units.Units.*;

public class CalibrateCommand extends SequentialCommandGroup {
    private ArmCalibration calibration;

    public CalibrateCommand(ArmCalibrationParams arm, Voltage maxVoltage, Consumer<ArmCalibration> consumer, Subsystem ... requirements) {
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
                        state -> Degrees.of(1),
                        Seconds.of(1),
                        DegreesPerSecond.of(25)),
                result -> {}
        )
                .andThen(new InstantCommand(() -> arm.setVoltage(Volts.of(0)), arm.requirements()));
    }
}
