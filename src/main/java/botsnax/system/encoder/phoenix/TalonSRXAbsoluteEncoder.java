package botsnax.system.encoder.phoenix;

import botsnax.system.encoder.AbsoluteEncoder;
import botsnax.system.encoder.EncoderSim;
import botsnax.system.motor.phoenix.TalonSRXMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static botsnax.system.motor.phoenix.TalonSRXMotor.ENCODER_RESOLUTION;
import static botsnax.system.motor.phoenix.TalonSRXMotor.VELOCITY_TIME_UNITS_PER_SECOND;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static java.lang.Math.round;

public class TalonSRXAbsoluteEncoder implements AbsoluteEncoder {
    private class Sim implements EncoderSim {
        @Override
        public void setSupplyVoltage(Voltage voltage) {
            motor.getSim().setSupplyVoltage(voltage);
        }

        @Override
        public void setInverted(boolean isInverted) {
            if (isInverted) {
                throw new UnsupportedOperationException("Can't invert TalonSRX encoder");
            }
        }

        @Override
        public void setRawPosition(Angle angle) {
            motor.get().getSimCollection().setPulseWidthPosition(
                    (int) round(angle.in(Rotations) * ENCODER_RESOLUTION));
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            motor.get().getSimCollection().setPulseWidthVelocity(
                    (int) round(velocity.in(RotationsPerSecond) * ENCODER_RESOLUTION / VELOCITY_TIME_UNITS_PER_SECOND));
        }
    }

    private final TalonSRXMotor motor;
    private final Sim sim;

    public TalonSRXAbsoluteEncoder(TalonSRXMotor motor) {
        this.motor = motor;
        this.sim = isSimulation() ? new Sim() : null;
    }

    @Override
    public EncoderSim getSim() {
        return sim;
    }

    @Override
    public double getZeroingValue() {
        return -motor.getAbsolutePosition();
    }

    @Override
    public void applySimZeroingValue(double value) {
        motor.get().getSimCollection().setPulseWidthPosition((int) value);
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
