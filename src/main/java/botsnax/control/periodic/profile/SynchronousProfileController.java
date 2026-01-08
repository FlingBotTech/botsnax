package botsnax.control.periodic.profile;

import botsnax.control.periodic.ApplyMode;
import botsnax.control.periodic.SetpointMode;
import botsnax.control.periodic.SynchronousController;
import botsnax.control.periodic.ScalarState;

import java.util.Optional;

public abstract class SynchronousProfileController<ValueT, SystemT, StateT extends ScalarState<ValueT>>
        extends SynchronousController<SystemT, StateT, SetpointMode<ValueT, SystemT, StateT>>
        implements ProfileController<ValueT, SystemT, StateT> {

    public SynchronousProfileController(
            SystemT system,
            StateManager<SystemT, StateT> stateManager,
            SetpointMode<ValueT, SystemT, StateT> mode) {
        super(system, stateManager, mode);
        update();
    }

    @Override
    public Optional<ValueT> getSetpoint() {
        return mode.get().getSetpoint();
    }

    @Override
    public void setProfile(Profile<ValueT, StateT> profile, ApplyMode<ValueT, SystemT> applyMode) {
        setMode(new ProfileMode<>(wrap(profile), wrap(applyMode)));
        update();
    }

    protected Profile<ValueT, StateT> wrap(Profile<ValueT, StateT> profile) {
        return profile;
    }

    protected ApplyMode<ValueT, SystemT> wrap(ApplyMode<ValueT, SystemT> mode) {
        return mode;
    }
}
