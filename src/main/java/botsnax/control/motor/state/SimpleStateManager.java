package botsnax.control.motor.state;

import botsnax.control.periodic.SynchronousController.StateManager;
import botsnax.system.motor.MotorState;
import botsnax.system.motor.MotorSystem;
import edu.wpi.first.units.measure.Time;

import java.util.function.Function;
import java.util.function.Supplier;

public class SimpleStateManager<SystemT, StateT> implements StateManager<SystemT, StateT> {
    private final Function<SystemT, StateT> getState;
    private StateT state = null;

    public SimpleStateManager(Function<SystemT, StateT> getState) {
        this.getState = getState;
    }

    @Override
    public void update(SystemT system) {
        state = getState.apply(system);
    }

    @Override
    public StateT getState(SystemT system) {
        return state;
    }

    public static StateManager<MotorSystem, MotorState> getDefaultAngular(Supplier<Time> timeSource) {
        return new SimpleStateManager<>(system -> system.getState(timeSource.get()));
    }
}
