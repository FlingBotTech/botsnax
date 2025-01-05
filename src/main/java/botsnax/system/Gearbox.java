package botsnax.system;

import botsnax.system.motor.MotorSystem;
import botsnax.system.encoder.AbsoluteEncoder;

public class Gearbox {
    private final MotorSystem motorSystem;
    private final AbsoluteEncoder outputEncoder;

    public Gearbox(MotorSystem motorSystem, AbsoluteEncoder outputEncoder) {
        this.motorSystem = motorSystem;
        this.outputEncoder = outputEncoder;
    }

    public MotorSystem getMotor() {
        return motorSystem;
    }

    public AbsoluteEncoder getOutputEncoder() {
        return outputEncoder;
    }
}