package botsnax.system.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

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
    public Angle getAngle() {
        return motor.getAngle().div(gearRatio);
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().div(gearRatio);
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setAngle(angle.times(gearRatio));
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public Voltage getVoltage() {
        return motor.getVoltage();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }
}
