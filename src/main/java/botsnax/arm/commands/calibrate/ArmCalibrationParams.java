package botsnax.arm.commands.calibrate;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.system.motor.MotorSystem;
import botsnax.system.motor.MotorState;
import botsnax.system.Gearbox;

import static edu.wpi.first.units.Units.Degrees;

public record ArmCalibrationParams(
        Gearbox gearbox,
        Angle range,
        Subsystem ... requirements
) {
    public MotorState getMotorState(Time time) {
        return gearbox.getMotor().getState(time);
    }

    public MotorSystem encoderMotorSystem() {
        return gearbox.getMotor().withEncoder(gearbox.getOutputEncoder(), false);
    }

    public MotorState getEncoderState(Time time) {
        return new MotorState(gearbox().getMotor().getVoltage(), time, gearbox.getOutputEncoder().getAngle(), gearbox.getOutputEncoder().getVelocity());
    }

    public void setVoltage(Voltage voltage) {
        gearbox.getMotor().setVoltage(voltage);
    }

    public Angle getAngle() {
        return gearbox.getOutputEncoder().getAngle();
    }

    public AngularVelocity getVelocity() {
        return gearbox.getOutputEncoder().getVelocity();
    }

    public void zeroPositions() {
        gearbox.getMotor().setAngle(Degrees.of(0));
        gearbox.getOutputEncoder().setAngle(Degrees.of(0));
    }
}
