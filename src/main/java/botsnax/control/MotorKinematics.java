package botsnax.control;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface MotorKinematics {
    Voltage getVoltageForVelocity(AngularVelocity velocity, MotorState state);
}
