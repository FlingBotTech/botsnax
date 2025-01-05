package botsnax.arm.commands;

import botsnax.arm.commands.calibrate.ArmCalibrationParams;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import botsnax.control.MotorController;
import botsnax.system.motor.MotorState;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.hal.HALUtil.getFPGATime;
import static edu.wpi.first.math.filter.LinearFilter.movingAverage;
import static edu.wpi.first.units.Units.*;
import static java.lang.Math.abs;

public class AwaitStabilityCommand extends Command {
    public record Result(Measure<Angle> angle, Measure<Angle> motorAngle, double voltage) {
    }

    private static final Measure<Time> minPhaseTime = Seconds.of(2);
    private static final Measure<Angle> stabilityThreshold = Degrees.of(0.02);

    private final ArmCalibrationParams arm;
    private final LinearFilter longAverageFilter;
    private final LinearFilter shortAverageFilter;
    private final LinearFilter motorAngleFilter;
    private final LinearFilter voltageAverageFilter;
    private final Supplier<MotorController> controllerSupplier;
    private MotorController motorController;
    private final Consumer<Result> consumer;

    private Measure<Time> startTime;
    private Result result;

    public AwaitStabilityCommand(ArmCalibrationParams arm, Supplier<MotorController> controllerSupplier, Consumer<Result> consumer) {
        this.arm = arm;
        this.controllerSupplier = controllerSupplier;
        this.consumer = consumer;

        longAverageFilter = movingAverage(80);
        shortAverageFilter = movingAverage(20);
        motorAngleFilter = movingAverage(20);
        voltageAverageFilter = movingAverage(20);
        startTime = Seconds.of(Double.POSITIVE_INFINITY);

        addRequirements(arm.requirements());
    }

    public AwaitStabilityCommand(ArmCalibrationParams arm, MotorController controller, Consumer<Result> consumer) {
        this(arm, () -> controller, consumer);
    }

    private static Measure<Time> getTime() {
        return Microseconds.of(getFPGATime());
    }

    @Override
    public void initialize() {
        motorController = controllerSupplier.get();
        startTime = getTime();
    }

    @Override
    public void execute() {
        Measure<Time> time = getTime().minus(startTime);
        MotorState motorState = arm.getMotorState(time);
        MotorState encoderState = arm.getEncoderState(time);
        double voltage = motorController.calculate(encoderState);

        arm.setVoltage(voltage);

        double angle = arm.getAngle().baseUnitMagnitude();
        double motorAngle = motorState.getAngle().baseUnitMagnitude();
        double shortAverage = shortAverageFilter.calculate(angle);
        double longAverage = longAverageFilter.calculate(angle);
        double voltageAverage = voltageAverageFilter.calculate(voltage);
        double motorAngleAverage = motorAngleFilter.calculate(motorAngle);

        if (time.gt(minPhaseTime)) {
            boolean isStable = abs(longAverage - shortAverage) < stabilityThreshold.baseUnitMagnitude();

            if (isStable) {
                result = new Result(
                        BaseUnits.Angle.of(shortAverage),
                        BaseUnits.Angle.of(motorAngleAverage),
                        voltageAverage);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return result != null;
    }

    @Override
    public void end(boolean interrupted) {
        consumer.accept(result);
    }
}
