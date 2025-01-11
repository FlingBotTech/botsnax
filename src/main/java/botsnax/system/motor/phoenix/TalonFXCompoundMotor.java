package botsnax.system.motor.phoenix;

import botsnax.system.motor.MotorSim;
import botsnax.system.motor.CompoundMotor;

public class TalonFXCompoundMotor extends CompoundMotor<TalonFXMotor> implements TalonFXMotorSystem {
    public TalonFXCompoundMotor(TalonFXMotor ... motors) {
        super(motors);
    }

    public MotorSim getSim() {
        return getPrimary().getSim();
    }
}
