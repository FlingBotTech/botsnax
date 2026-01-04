package botsnax.control.periodic.profile;

import botsnax.control.periodic.ApplyMode;
import botsnax.control.periodic.SetpointMode;
import botsnax.control.periodic.ScalarState;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class ProfileMode<ValueT, SystemT, StateT extends ScalarState<ValueT>> implements SetpointMode<ValueT, SystemT, StateT> {
    private final Profile<ValueT, StateT> profile;
    private final ApplyMode<ValueT, SystemT> applyMode;

    private final AtomicReference<ValueT> setpoint;

    public ProfileMode(Profile<ValueT, StateT> profile, ApplyMode<ValueT, SystemT> applyMode) {
        this.profile = profile;
        this.applyMode = applyMode;
        setpoint = new AtomicReference<>();
    }

    @Override
    public void update(SystemT system, StateT state) {
        ValueT nextSetpoint = profile.getValue(state, Optional.ofNullable(setpoint.get()));
        applyMode.apply(nextSetpoint, system);
        setpoint.set(nextSetpoint);
    }

    @Override
    public Optional<ValueT> getSetpoint() {
        return Optional.of(setpoint.get());
    }
}
