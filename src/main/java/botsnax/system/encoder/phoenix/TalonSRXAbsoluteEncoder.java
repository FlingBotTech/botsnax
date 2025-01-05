package botsnax.system.encoder.phoenix;

import botsnax.system.encoder.AbsoluteEncoder;
import botsnax.system.motor.phoenix.TalonSRXMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

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
    public Measure<Angle> getAngle() {
        return motor.getAbsoluteAngle();
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public void setAngle(Measure<Angle> angle) {
        motor.setAbsoluteAngle(angle);
    }
}
