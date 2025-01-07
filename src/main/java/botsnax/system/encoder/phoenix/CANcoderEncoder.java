package botsnax.system.encoder.phoenix;

import botsnax.system.encoder.AbsoluteEncoder;
import botsnax.util.phoenix.CANcoderUtil;
import botsnax.util.phoenix.PhoenixUtil;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Rotations;

public class CANcoderEncoder implements AbsoluteEncoder {
    private final CANcoder cancoder;

    public CANcoderEncoder(CANcoder cancoder) {
        this.cancoder = cancoder;
    }

    @Override
    public double getZeroingValue() {
        return CANcoderUtil.getOffsetToZeroCurrentPosition(cancoder).in(Rotations);
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
