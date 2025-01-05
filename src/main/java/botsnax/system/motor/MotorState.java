package botsnax.system.motor;

import edu.wpi.first.units.*;

public class MotorState {
    private final double voltage;
    private final Measure<Time> time;
    private final Measure<Angle> angle;
    private final Measure<Velocity<Angle>> velocity;

    public MotorState(double voltage, Measure<Time> time, Measure<Angle> angle, Measure<Velocity<Angle>> velocity) {
        this.voltage = voltage;
        this.time = time;
        this.angle = angle;
        this.velocity = velocity;
    }

    public double getVoltage() {
        return voltage;
    }

    public Measure<Time> getTime() {
        return time;
    }

    public Measure<Angle> getAngle() {
        return angle;
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return velocity;
    }
}
