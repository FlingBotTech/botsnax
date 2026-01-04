package botsnax.control.periodic.profile;

import botsnax.control.periodic.ApplyMode;
import botsnax.control.periodic.Controller;
import botsnax.control.periodic.SetpointMode;
import botsnax.control.periodic.ScalarState;

import java.util.Optional;

public interface ProfileController<ValueT, SystemT, StateT extends ScalarState<ValueT>>
        extends Controller<SystemT, StateT, SetpointMode<ValueT, SystemT, StateT>> {
    Optional<ValueT> getSetpoint();
    void setProfile(Profile<ValueT, StateT> profile, ApplyMode<ValueT, SystemT> applyMode);
}
