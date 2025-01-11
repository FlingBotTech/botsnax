package botsnax.system.encoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface EncoderSim {
    void setSupplyVoltage(Voltage voltage);
    void setInverted(boolean isInverted);
    void setRawPosition(Angle angle);
    void setVelocity(AngularVelocity velocity);
}
