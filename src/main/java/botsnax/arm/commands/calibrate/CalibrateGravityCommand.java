package botsnax.arm.commands.calibrate;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.*;
import botsnax.control.MotorController;
import botsnax.arm.commands.AwaitStabilityCommand;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.floor;

public class CalibrateGravityCommand extends SequentialCommandGroup {
    private static final Measure<Angle> increment = Degrees.of(15);

    private final ArmCalibrationParams arm;
    private final ArmPositionResults armPositionResults = new ArmPositionResults();

    private ArmCalibration calibration = null;

    public CalibrateGravityCommand(
            ArmCalibrationParams arm,
            Measure<Angle> startAngle,
            Supplier<ArmCalibration> calibrationSupplier,
            Consumer<ArmCalibration> calibrationConsumer) {
        this.arm = arm;

        addCommands(new InstantCommand(() -> calibration = calibrationSupplier.get(), arm.requirements()));
        addPositionCommands(startAngle);
        addCommands(new AwaitStabilityCommand(arm, () -> createController(startAngle), result -> {})
                .andThen(solve(calibrationConsumer, arm.requirements())));
    }

    private void addPositionCommands(Measure<Angle> startAngle) {
        int phases = (int) floor(arm.range().times(0.9).minus(startAngle).baseUnitMagnitude() /
                increment.baseUnitMagnitude());

        for (int i = 0; i < phases; i++) {
            addCommands(stabilizeAtAngle(startAngle.plus(increment.times(i))));
        }
    }

    private Command stabilizeAtAngle(Measure<Angle> angle) {
        return new AwaitStabilityCommand(arm, () -> createController(angle), result -> {
            armPositionResults.add(result);

            if (result.voltage() > calibration.gravityController().gain()) {
                calibration = calibration.withGravity(new ArmGravityController(result.voltage(), angle));
            }
        });
    }

    private Command solve(Consumer<ArmCalibration> calibrationConsumer, Subsystem... requirements) {
        return new InstantCommand(() -> calibrationConsumer.accept(
                calibration
                        .withGravity(armPositionResults.solveForGravity())
                        .withGearRatio(armPositionResults.solveForGearRatio())),
                requirements
        );
    }

    private MotorController createController(Measure<Angle> angle) {
        return calibration.createController(
                state -> angle,
                Seconds.of(0.25),
                DegreesPerSecond.of(60));
    }
}
