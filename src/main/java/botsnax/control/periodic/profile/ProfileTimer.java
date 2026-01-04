package botsnax.control.periodic.profile;

import botsnax.control.periodic.ScalarState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public class ProfileTimer {
    public interface TimedProfile<ValueT, StateT> {
        ValueT apply(StateT state, Optional<ValueT> priorValue, ProfileTimer timer);
    }

    private final Supplier<Time> timeSupplier;
    private Time startTime = null;
    private Time lastTime = null;
    private Time timeSinceLastUpdate = Seconds.zero();
    private Time elapsedTime = Seconds.zero();

    public ProfileTimer(Supplier<Time> timeSupplier) {
        this.timeSupplier = timeSupplier;
    }

    void update() {
        Time now = timeSupplier.get();

        if (startTime == null) {
            startTime = now;
        }

        if (lastTime == null) {
            lastTime = now;
        }

        timeSinceLastUpdate = now.minus(lastTime);
        elapsedTime = now.minus(startTime);

        lastTime = now;
    }

    public Time getTimeSinceLastUpdate() {
        return timeSinceLastUpdate;
    }

    public Time getElapsedTime() {
        return elapsedTime;
    }

    public static <ValueT, StateT extends ScalarState<ValueT>> Profile<ValueT, StateT> of(Supplier<Time> timeSupplier, TimedProfile<ValueT, StateT> profile) {
        final ProfileTimer timer = new ProfileTimer(timeSupplier);

        return (state, priorValue) -> {
            timer.update();
            return profile.apply(state, priorValue, timer);
        };
    }

    public static <StateT extends ScalarState<Angle>> Profile<Angle, StateT> ofConstantAngularVelocity(Supplier<Time> timeSupplier, AngularVelocity velocity) {
        return of(timeSupplier, (state, priorValue, timer) ->
                priorValue.orElse(state.getValue()).plus(velocity.times(timer.getTimeSinceLastUpdate())));
    }
}
