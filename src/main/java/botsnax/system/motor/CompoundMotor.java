package botsnax.system.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.Consumer;

import static botsnax.util.DCMotorUtil.withMotorCount;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class CompoundMotor<T extends MotorSystem> implements MotorSystem {
    private class Sim implements MotorSim {
        @Override
        public Voltage getMotorVoltage() {
            return getPrimary().getSim().getMotorVoltage();
        }

        @Override
        public void setSupplyVoltage(Voltage voltage) {
            getPrimary().getSim().setSupplyVoltage(voltage);
        }

        @Override
        public void setInverted(boolean isInverted) {
            getPrimary().getSim().setInverted(isInverted);
        }

        @Override
        public void setRawPosition(Angle angle) {
            for(T motor : motors) {
                motor.getSim().setRawPosition(angle);
            }
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            for (T motor : motors) {
                motor.getSim().setVelocity(velocity);
            }
        }
    }

    private final T[] motors;
    private final MotorSim sim;
    private final DCMotor dcMotor;

    public CompoundMotor(T[] motors) {
        this.motors = motors;

        sim = isSimulation() ? new Sim() : null;
        dcMotor = withMotorCount(getPrimary().getDCMotor(), motors.length);
    }

    public T getPrimary() {
        return motors[0];
    }

    @Override
    public int getDeviceID() {
        throw new UnsupportedOperationException();
    }

    @Override
    public MotorSim getSim() {
        return sim;
    }

    @Override
    public DCMotor getDCMotor() {
        return dcMotor;
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
    public Angle getAngle() {
        return getPrimary().getAngle();
    }

    @Override
    public AngularVelocity getVelocity() {
        return getPrimary().getVelocity();
    }

    @Override
    public Voltage getVoltage() {
        return getPrimary().getVoltage();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        for (MotorSystem motor : motors) {
            motor.setVoltage(voltage);
        }
    }

    @Override
    public void stop() {
        forEach(MotorSystem::stop);
    }

    @Override
    public void setAngle(Angle angle) {
        forEach(motor -> motor.setAngle(angle));
    }

    public void forEach(Consumer<T> consumer) {
        for (T motor : motors) {
            consumer.accept(motor);
        }
    }
}
