package botsnax.swerve.sim;

import botsnax.system.Gearbox;
import botsnax.system.encoder.AbsoluteEncoder;
import botsnax.system.motor.MotorSim;
import botsnax.system.motor.MotorSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class PerfectSteering implements MotorSystem, AbsoluteEncoder, MotorSim {
    private Angle angle = Radians.zero();

    @Override
    public void applySimZeroingValue(double value) {
        angle = Radians.zero();
    }

    @Override
    public double getZeroingValue() {
        return 0;
    }

    @Override
    public void setVoltage(Voltage voltage) { }

    @Override
    public Voltage getVoltage() {
        return Volts.of(0);
    }

    @Override
    public void stop() { }

    @Override
    public MotorSim getSim() {
        return this;
    }

    @Override
    public Angle getAngle() {
        return angle;
    }

    @Override
    public AngularVelocity getVelocity() {
        return RadiansPerSecond.zero();
    }

    @Override
    public void setAngle(Angle angle) {
        this.angle = angle;
    }

    @Override
    public DCMotor getDCMotor() {
        return DCMotor.getVex775Pro(1);
    }

    @Override
    public int getDeviceID() {
        return 0;
    }

    @Override
    public void setCoastOnIdle() { }

    @Override
    public void setBrakeOnIdle() { }

    @Override
    public void setSupplyVoltage(Voltage voltage) { }

    @Override
    public void setInverted(boolean isInverted) { }

    @Override
    public void setRawPosition(Angle angle) {
        // Simulation will call this -- ignore.
    }

    @Override
    public void setVelocity(AngularVelocity velocity) { }

    @Override
    public Voltage getMotorVoltage() {
        return Volts.zero();
    }

    public static Gearbox createGearbox() {
        PerfectSteering steering = new PerfectSteering();

        return new Gearbox(steering, steering);
    }
}
