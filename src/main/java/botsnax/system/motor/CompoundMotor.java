package botsnax.system.motor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import java.util.function.Consumer;

public class CompoundMotor<T extends MotorSystem> implements MotorSystem {
    private final T[] motors;

    public CompoundMotor(T[] motors) {
        this.motors = motors;
    }

    public T getPrimary() {
        return motors[0];
    }

    @Override
    public int getDeviceID() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setCoastOnIdle() {
        for (T motor : motors) {
            motor.setCoastOnIdle();
        }
    }

    @Override
    public void setBrakeOnIdle() {
        for (T motor : motors) {
            motor.setBrakeOnIdle();
        }
    }

    @Override
    public Measure<Angle> getAngle() {
        return getPrimary().getAngle();
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return getPrimary().getVelocity();
    }

    @Override
    public double getVoltage() {
        return getPrimary().getVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        for (MotorSystem motor : motors) {
            motor.setVoltage(voltage);
        }
    }

    @Override
    public void stop() {
        forEach(MotorSystem::stop);
    }

    @Override
    public void setAngle(Measure<Angle> angle) {
        forEach(motor -> motor.setAngle(angle));
    }

    public void forEach(Consumer<T> consumer) {
        for (T motor : motors) {
            consumer.accept(motor);
        }
    }
}
