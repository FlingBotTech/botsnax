package botsnax.util;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;

import java.util.function.Consumer;

import static com.ctre.phoenix6.Utils.getCurrentTimeSeconds;
import static edu.wpi.first.units.Units.Seconds;

public class SimRunner {
    private Time lastSimTime = null;

    public void start(Consumer<Time> updater, Time period) {
        Notifier notifier = new Notifier(() -> {
            final Time currentTime = Seconds.of(getCurrentTimeSeconds());

            if (lastSimTime != null) {
                Time deltaTime = currentTime.minus(lastSimTime);
                updater.accept(deltaTime);
            }

            lastSimTime = currentTime;
        });

        notifier.startPeriodic(period.in(Seconds));
    }
}
