package botsnax.system.motor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class GearedMotorSystem implements MotorSystem {
    private final MotorSystem motor;
    private final double gearRatio;

    public GearedMotorSystem(MotorSystem motor, double gearRatio) {
        this.motor = motor;
        this.gearRatio = gearRatio;
    }

    @Override
    public int getDeviceID() {
        return motor.getDeviceID();
    }

    @Override
    public void setBrakeOnIdle() {
        motor.setBrakeOnIdle();
    }

    @Override
    public void setCoastOnIdle() {
        motor.setBrakeOnIdle();
    }

    @Override
    public Measure<Angle> getAngle() {
        return motor.getAngle().divide(gearRatio);
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return motor.getVelocity().divide(gearRatio);
    }

    @Override
    public void setAngle(Measure<Angle> angle) {
        motor.setAngle(angle.times(gearRatio));
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public double getVoltage() {
        return motor.getVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
