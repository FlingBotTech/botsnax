package botsnax.elevator.calibrate;

import botsnax.commands.MotorControllerCommand;
import botsnax.commands.calibrate.AwaitStableValueCommand;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

public class CalibrateGravityCommand extends SequentialCommandGroup {
    private final ElevatorCalibrationParams elevator;
    private ElevatorGravityController gravityController = new ElevatorGravityController(Volts.of(0));

    public CalibrateGravityCommand(
            ElevatorCalibrationParams elevator,
            Voltage maxVoltage,
            Consumer<Voltage> gravityVoltage) {
        this.elevator = elevator;

        Command calibrateGravity = getProportionalController(maxVoltage, elevator.height().div(2))
                .raceWith(new AwaitStableValueCommand(
                        Seconds.of(1),
                        0.005,
                        () -> elevator.motor().getVoltage().in(Volts),
                        result -> {
                            gravityController = new ElevatorGravityController(Volts.of(result));
                            gravityVoltage.accept(Volts.of(result));
                        }
                ));

        Command park = getProportionalController(maxVoltage.div(2), elevator.height().times(0.05))
                .raceWith(new AwaitStableValueCommand(
                        Seconds.of(1),
                        0.1,
                        () -> elevator.motor().getState(Seconds.zero()).getAngle().in(Degrees),
                        result -> { }
                ));

        addCommands(calibrateGravity, park);
    }

    private Command getProportionalController(Voltage maxVoltage, Distance targetPosition) {
        Voltage gain = maxVoltage.div(targetPosition.baseUnitMagnitude());

        return new MotorControllerCommand(
                elevator.motor(),
                () -> state -> {
                    Distance currentPosition = elevator.linearAngularConversion().getDistance(state.getAngle());
                    Distance error = targetPosition.minus(currentPosition);

                    return gain.times(error.baseUnitMagnitude()).plus(gravityController.calculate(state));
                },
                elevator.requirements()
        );
    }
}
