package botsnax.system.encoder.phoenix;

import botsnax.system.encoder.AbsoluteEncoder;
import botsnax.system.motor.phoenix.TalonSRXMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class TalonSRXAbsoluteEncoder implements AbsoluteEncoder {
    private final TalonSRXMotor motor;

    public TalonSRXAbsoluteEncoder(TalonSRXMotor motor) {
        this.motor = motor;
    }

    @Override
    public double getZeroingValue() {
        return -motor.getAbsolutePosition();
    }

    @Override
    public Angle getAngle() {
        return motor.getAbsoluteAngle();
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setAbsoluteAngle(angle);
    }
}
