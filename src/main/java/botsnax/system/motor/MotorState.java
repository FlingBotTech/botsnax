package botsnax.system.motor;

import botsnax.control.motor.state.AngleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class MotorState implements AngleState {
    private final Voltage voltage;
    private final Time time;
    private final Angle angle;
    private final AngularVelocity velocity;

    public MotorState(Voltage voltage, Time time, Angle angle, AngularVelocity velocity) {
        this.voltage = voltage;
        this.time = time;
        this.angle = angle;
        this.velocity = velocity;
    }

    @Override
    public Angle getValue() {
        return angle;
    }

    public Voltage getVoltage() {
        return voltage;
    }

    public Time getTime() {
        return time;
    }

    public Angle getAngle() {
        return angle;
    }

    public AngularVelocity getVelocity() {
        return velocity;
    }
}
