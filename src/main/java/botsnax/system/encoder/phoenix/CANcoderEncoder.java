package botsnax.system.encoder.phoenix;

import botsnax.system.encoder.AbsoluteEncoder;
import botsnax.system.encoder.EncoderSim;
import botsnax.util.phoenix.CANcoderUtil;
import botsnax.util.phoenix.PhoenixUtil;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class CANcoderEncoder implements AbsoluteEncoder {
    private class Sim implements EncoderSim {
        @Override
        public void setSupplyVoltage(Voltage voltage) {
            cancoder.getSimState().setSupplyVoltage(voltage);
        }

        @Override
        public void setInverted(boolean isInverted) {
            cancoder.getSimState().Orientation = isInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        }

        @Override
        public void setRawPosition(Angle angle) {
            cancoder.getSimState().setRawPosition(angle);
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            cancoder.getSimState().setVelocity(velocity);
        }
    }

    private final CANcoder cancoder;
    private final Sim sim;

    public CANcoderEncoder(CANcoder cancoder) {
        this.cancoder = cancoder;
        this.sim = isSimulation() ? new Sim() : null;
    }

    @Override
    public EncoderSim getSim() {
        return sim;
    }

    @Override
    public double getZeroingValue() {
        return CANcoderUtil.getOffsetToZeroCurrentPosition(cancoder).in(Rotations);
    }

    @Override
    public void applySimZeroingValue(double value) {
        cancoder.getSimState().setRawPosition(value);
    }

    @Override
    public Angle getAngle() {
        return cancoder.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return cancoder.getVelocity().getValue();
    }

    @Override
    public void setAngle(Angle angle) {
        PhoenixUtil.setAndValidatePosition(cancoder, angle);
    }
}
