package botsnax.control.motor;

import botsnax.control.periodic.profile.ProfileController;
import botsnax.control.periodic.ScalarState;
import botsnax.system.motor.MotorSystem;

public interface MotorProfileController<ValueT, StateT extends ScalarState<ValueT>>
        extends ProfileController<ValueT, MotorSystem, StateT> {
    void stop();
}
