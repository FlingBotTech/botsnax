package botsnax.commands.calibrate;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.hal.HALUtil.getFPGATime;
import static edu.wpi.first.math.filter.LinearFilter.movingAverage;
import static edu.wpi.first.units.Units.Microseconds;
import static java.lang.Math.abs;

public class AwaitStableValueCommand extends Command {
    private final Measure<Time> minPhaseTime;
    private final double stabilityThreshold;

    private final Supplier<Double> valueSupplier;
    private final Consumer<Double> resultConsumer;
    private final LinearFilter longAverageFilter;
    private final LinearFilter shortAverageFilter;
    private Measure<Time> startTime;
    private Double result;

    public AwaitStableValueCommand(
            Measure<Time> minPhaseTime,
            double stabilityThreshold,
            Supplier<Double> valueSupplier,
            Consumer<Double> resultConsumer,
            Subsystem ... requirements) {
        this.minPhaseTime = minPhaseTime;
        this.stabilityThreshold = stabilityThreshold;
        this.valueSupplier = valueSupplier;
        this.resultConsumer = resultConsumer;

        longAverageFilter = movingAverage(80);
        shortAverageFilter = movingAverage(20);

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        startTime = getTime();
    }

    private static Measure<Time> getTime() {
        return Microseconds.of(getFPGATime());
    }

    @Override
    public void execute() {
        Measure<Time> time = getTime().minus(startTime);
        double value = valueSupplier.get();
        double shortAverage = shortAverageFilter.calculate(value);
        double longAverage = longAverageFilter.calculate(value);

        if (time.gt(minPhaseTime)) {
            boolean isStable = abs(longAverage - shortAverage) < stabilityThreshold;

            if (isStable) {
                result = value;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (result != null);
    }

    @Override
    public void end(boolean interrupted) {
        if (result != null) {
            resultConsumer.accept(result);
        }
    }
}
