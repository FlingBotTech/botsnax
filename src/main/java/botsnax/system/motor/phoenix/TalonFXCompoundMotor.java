package botsnax.system.motor.phoenix;

import com.ctre.phoenix6.sim.TalonFXSimState;
import botsnax.system.motor.CompoundMotor;

public class TalonFXCompoundMotor extends CompoundMotor<TalonFXMotor> implements TalonFXMotorSystem {
    public TalonFXCompoundMotor(TalonFXMotor ... motors) {
        super(motors);
    }

    public TalonFXSimState getSimState() {
        return getPrimary().getSimState();
    }
}
