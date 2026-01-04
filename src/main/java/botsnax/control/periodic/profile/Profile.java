package botsnax.control.periodic.profile;

import botsnax.control.periodic.ScalarState;

import java.util.Optional;

public interface Profile<ValueT, StateT extends ScalarState<ValueT>> {
    ValueT getValue(StateT state, Optional<ValueT> priorValue);
}
