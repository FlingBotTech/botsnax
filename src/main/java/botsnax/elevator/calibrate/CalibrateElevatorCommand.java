package botsnax.elevator.calibrate;

import botsnax.commands.calibrate.CalibrateVelocityCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Volts;

public class CalibrateElevatorCommand extends SequentialCommandGroup {
    private ElevatorGravityController gravityController;

    public CalibrateElevatorCommand(ElevatorCalibrationParams elevator, Consumer<ElevatorCalibration> consumer) {
        addCommands(
                new CalibrateGravityCommand(
                        elevator,
                        Volts.of(6),
                        gravityVoltage -> gravityController = new ElevatorGravityController(gravityVoltage)
                ),
                new CalibrateVelocityCommand(
                        elevator.motor(),
                        elevator.linearAngularConversion().getAngle(elevator.height()),
                        Volts.of(8),
                        () -> gravityController,
                        velocityCalibration -> consumer.accept(new ElevatorCalibration(gravityController, velocityCalibration)),
                        elevator.requirements()
                )
        );
    }
}
