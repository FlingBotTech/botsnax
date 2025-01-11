package botsnax.flywheel;

import botsnax.system.VoltageActuator;
import botsnax.system.encoder.Encoder;
import botsnax.system.motor.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;

public interface Flywheel extends VoltageActuator, Encoder, IdleMode {
    DCMotor getDCMotor();
    int getDeviceID();
}
