package botsnax.arm.commands.calibrate;

import botsnax.commands.MotorControllerCommand;
import botsnax.commands.calibrate.VelocityCalibrationData;
import botsnax.control.CompositeMotorController;
import botsnax.control.MotorController;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static botsnax.control.MotorControllerFactory.createConstantController;
import static edu.wpi.first.units.Units.Volts;

public class CalibrateVelocityCommand extends SequentialCommandGroup {
    private static final int PHASES = 3;
    private static final double MAX_FRACTION_OF_AVAILABLE_VOLTAGE = 0.15;

    private final ArmCalibrationParams arm;
    private final VelocityCalibrationData data = new VelocityCalibrationData();
    private ArmCalibration calibration;

    public CalibrateVelocityCommand(
            ArmCalibrationParams arm,
            Voltage maxVoltage,
            Supplier<ArmCalibration> calibrationSupplier,
            Consumer<ArmCalibration> calibrationConsumer) {
        this.arm = arm;

        addCommands(new InstantCommand(() -> calibration = calibrationSupplier.get(), arm.requirements()));

        for (int i = 1; i <= PHASES; i++) {
            addCommands(
                    getRunAtVoltageCommand(i, maxVoltage),
                    getRunAtVoltageCommand(-i, maxVoltage)
            );
        }

        addCommands(
                new InstantCommand(() -> calibrationConsumer.accept(calibration.withVelocity(
                        data.solvePositive(),
                        data.solveNegative())),
                arm.requirements())
        );
    }

    private Voltage getVoltage(int increment, Voltage maxVoltage) {
        Voltage maxGravityVoltage = calibration.gravityController().gain();
        Voltage maxTestVoltage = maxVoltage.minus(maxGravityVoltage).times(MAX_FRACTION_OF_AVAILABLE_VOLTAGE);

        return maxTestVoltage.times(increment).div(PHASES);
    }

    private Command getRunAtVoltageCommand(int increment, Voltage maxVoltage, Subsystem ... requirements) {
        final List<AngularVelocity> velocities = new ArrayList<>();
        BooleanSupplier isOutOfRange = increment > 0 ?
                () -> arm.getAngle().gt(arm.range().times(0.8)) :
                () -> arm.getAngle().lt(arm.range().times(0.2));
        Supplier<MotorController> controllerSupplier = () -> new CompositeMotorController(
                calibration.gravityController(),
                createConstantController(getVoltage(increment, maxVoltage)),
                state -> {
                    velocities.add(state.getVelocity());
                    return Volts.of(0);
                });

        return new MotorControllerCommand(arm.encoderMotorSystem(), controllerSupplier, arm.requirements())
                .until(isOutOfRange)
                .andThen(new InstantCommand(() ->
                        data.add(summarize(velocities), getVoltage(increment, maxVoltage))));
    }

    private AngularVelocity summarize(List<AngularVelocity> velocities) {
        double average = velocities
                .subList(velocities.size() / 3, 2 * velocities.size() / 3)
                .stream()
                .mapToDouble(Measure::baseUnitMagnitude)
                .average()
                .orElseThrow();

        return BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(average);
    }
}
