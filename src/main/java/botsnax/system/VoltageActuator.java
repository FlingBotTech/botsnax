package botsnax.system;

import edu.wpi.first.units.measure.Voltage;

public interface VoltageActuator {
    void setVoltage(Voltage voltage);
    Voltage getVoltage();
    void stop();
}
