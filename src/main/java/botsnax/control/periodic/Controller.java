package botsnax.control.periodic;

public interface Controller<SystemT, StateT, ModeT extends Controller.Mode<SystemT, StateT>> extends Periodic {
    interface Mode<SystemT, StateT> {
        void update(SystemT system, StateT state);
    }

    SystemT getSystem();
    StateT getState();
    void setMode(ModeT mode);
}
