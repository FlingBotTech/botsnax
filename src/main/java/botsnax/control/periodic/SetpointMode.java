package botsnax.control.periodic;

import botsnax.control.periodic.Controller.Mode;

import java.util.Optional;

public interface SetpointMode<ValueT, SystemT, StateT> extends Mode<SystemT, StateT> {
    Optional<ValueT> getSetpoint();
}
