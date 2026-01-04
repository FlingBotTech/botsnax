package botsnax.control.periodic;

import java.util.concurrent.atomic.AtomicReference;

public class SynchronousController<SystemT, StateT, ModeT extends Controller.Mode<SystemT, StateT>> implements Controller<SystemT, StateT, ModeT> {
    public interface StateManager<SystemT, StateT> {
        void update(SystemT system);
        StateT getState(SystemT system);
    }

    protected final SystemT system;
    protected final StateManager<SystemT, StateT> stateManager;
    protected final AtomicReference<ModeT> mode;

    public SynchronousController(SystemT system, StateManager<SystemT, StateT> stateManager, ModeT mode) {
        this.system = system;
        this.stateManager = stateManager;
        this.mode = new AtomicReference<>(mode);
    }

    @Override
    public SystemT getSystem() {
        return system;
    }

    @Override
    public StateT getState() {
        return stateManager.getState(system);
    }

    @Override
    public void setMode(ModeT mode) {
        this.mode.set(wrap(mode));
    }

    @Override
    public void update() {
        stateManager.update(system);
        mode.get().update(system, getState());
    }

    protected ModeT wrap(ModeT mode) {
        return mode;
    }
}
