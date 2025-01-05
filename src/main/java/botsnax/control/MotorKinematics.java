package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.system.motor.MotorState;

public interface MotorKinematics {
    double getVoltageForVelocity(Measure<Velocity<Angle>> velocity, MotorState state);
}
