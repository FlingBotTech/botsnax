package botsnax.control.motor;

import botsnax.control.periodic.Controller;
import edu.wpi.first.units.measure.Time;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class IntakeMode<SystemT, StateT> implements Controller.Mode<SystemT, StateT> {
    public interface Logger {
        void log(String name, String status);
    }

    private final String name;
    private final Consumer<SystemT> start;
    private final Consumer<SystemT> stop;
    private final Function<StateT, Boolean> checkFinished;
    private final Logger logger;
    private final Supplier<Time> timeSupplier;
    private final Time timeout;
    private Time startTime = null;
    private boolean isStarted = false;
    private boolean isFinished = false;

    public IntakeMode(String name, Consumer<SystemT> start, Consumer<SystemT> stop, Function<StateT, Boolean> checkFinished, Logger logger, Time timeout, Supplier<Time> timeSupplier) {
        this.name = name;
        this.start = start;
        this.stop = stop;
        this.checkFinished = checkFinished;
        this.timeout = timeout;
        this.logger = logger;
        this.timeSupplier = timeSupplier;
    }

    @Override
    public void update(SystemT system, StateT state) {
        if (!isFinished) {
            if (checkFinished.apply(state)) {
                finish(system, "Finished");
            } else if (!isStarted) {
                isStarted = true;
                startTime = timeSupplier.get();
                start.accept(system);
                logger.log(name, "Started");
            } else if (isTimedOut()) {
                finish(system, "Timeout");
            }
        }
    }

    private void finish(SystemT system, String status) {
        stop.accept(system);
        isFinished = true;
        logger.log(name, status);
    }

    private boolean isTimedOut() {
        return (startTime != null) && (timeSupplier.get().minus(startTime).gte(timeout));
    }
}