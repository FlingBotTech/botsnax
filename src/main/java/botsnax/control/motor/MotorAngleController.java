package botsnax.control.motor;

import botsnax.control.motor.state.SimpleStateManager;
import botsnax.system.motor.MotorState;
import botsnax.system.motor.MotorSystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

import java.util.function.Supplier;

import static edu.wpi.first.hal.HALUtil.getFPGATime;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Rotations;
import static java.lang.Double.NaN;

public class MotorAngleController extends SynchronousMotorProfileController<Angle, MotorState> {
    public MotorAngleController(MotorSystem motorSystem, StateManager<MotorSystem, MotorState> stateManager) {
        super(motorSystem, stateManager);
    }

    public MotorAngleController(MotorSystem motorSystem, Supplier<Time> timeSource) {
        this(motorSystem, SimpleStateManager.getDefaultAngular(timeSource));
    }

    public MotorAngleController(MotorSystem motorSystem) {
        this(motorSystem, () -> Microseconds.of(getFPGATime()));
    }

    @Override
    public Angle getSetpointError() {
        return getSetpoint()
                .map(setpoint -> setpoint.minus(getState().getAngle()))
                .orElse(Rotations.of(NaN));
    }
}
