package botsnax.flywheel;

import botsnax.system.VoltageActuator;
import botsnax.system.encoder.Encoder;
import botsnax.system.motor.IdleMode;

public interface Flywheel extends VoltageActuator, Encoder, IdleMode {
    int getDeviceID();
}
