package botsnax.commands;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

public class FiniteWaitUntilCommand {
    private static long timeoutCounter = 0;
    private static Logger logger = new DefaultLogger();

    public interface Logger {
        void onTimeout(String name);
    }

    public static class DefaultLogger implements Logger {
        IntegerPublisher timeoutCounterPub;

        public DefaultLogger() {
            timeoutCounterPub = NetworkTableInstance.getDefault().getIntegerTopic("TimeoutCounter").publish();
        }

        @Override
        public void onTimeout(String name) {
            timeoutCounterPub.set(timeoutCounter);
        }
    }

    public static long getTimeoutCounter() {
        return timeoutCounter;
    }

    public static void setLogger(Logger logger) {
        FiniteWaitUntilCommand.logger = logger;
    }

    public static Command of(BooleanSupplier condition, Time timeout, String name) {
        return new WaitUntilCommand(condition)
                .raceWith(new WaitCommand(timeout)
                        .andThen(new InstantCommand(() -> {
                            timeoutCounter++;
                            System.err.println("Timed out waiting for " + name + ". Total timeouts: " + timeoutCounter);
                            logger.onTimeout(name);
                        })));
    }
}
