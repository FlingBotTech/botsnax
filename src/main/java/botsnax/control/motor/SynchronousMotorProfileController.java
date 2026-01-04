package botsnax.control.motor;

import botsnax.control.periodic.SetpointMode;
import botsnax.control.periodic.ScalarState;
import botsnax.control.periodic.profile.SynchronousProfileController;
import botsnax.system.motor.MotorSystem;

import java.util.Optional;

public abstract class SynchronousMotorProfileController<ValueT, StateT extends ScalarState<ValueT>>
        extends SynchronousProfileController<ValueT, MotorSystem, StateT>
        implements MotorProfileController<ValueT, StateT> {

    private static class Idle<ValueT, StateT> implements SetpointMode<ValueT, MotorSystem, StateT> {
        @Override
        public void update(MotorSystem motorSystem, StateT state) {
            if (motorSystem.getVoltage().baseUnitMagnitude() != 0) {
                motorSystem.stop();
            }
        }

        @Override
        public Optional<ValueT> getSetpoint() {
            return Optional.empty();
        }
    }

    public SynchronousMotorProfileController(MotorSystem motorSystem, StateManager<MotorSystem, StateT> stateManager) {
        super(motorSystem, stateManager, new Idle<>());
    }

    @Override
    public final void stop() {
        setMode(new Idle<>());
    }
}
