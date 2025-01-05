package botsnax.system.encoder.phoenix;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.system.encoder.AbsoluteEncoder;
import botsnax.util.phoenix.CANcoderUtil;
import botsnax.util.phoenix.PhoenixUtil;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

public class CANcoderEncoder implements AbsoluteEncoder {
    private final CANcoder cancoder;

    public CANcoderEncoder(CANcoder cancoder) {
        this.cancoder = cancoder;
    }

    @Override
    public double getZeroingValue() {
        return CANcoderUtil.getOffsetToZeroCurrentPosition(cancoder);
    }

    @Override
    public Measure<Angle> getAngle() {
        return Rotations.of(cancoder.getPosition().getValue());
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return Rotations.per(Second).of(cancoder.getVelocity().getValue());
    }

    @Override
    public void setAngle(Measure<Angle> angle) {
        PhoenixUtil.setAndValidatePosition(cancoder, angle);
    }
}
