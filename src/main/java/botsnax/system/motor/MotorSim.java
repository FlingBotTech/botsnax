package botsnax.system.motor;

import botsnax.system.encoder.EncoderSim;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public interface MotorSim extends EncoderSim {
    MotorSim STUB = new MotorSim() {
        @Override
        public void setSupplyVoltage(Voltage voltage) { }

        @Override
        public void setInverted(boolean isInverted) { }

        @Override
        public void setRawPosition(Angle angle) { }

        @Override
        public void setVelocity(AngularVelocity velocity) { }

        @Override
        public Voltage getMotorVoltage() { return Volts.of(0); }
    };

    Voltage getMotorVoltage();
}
