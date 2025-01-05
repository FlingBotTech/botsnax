package botsnax.arm.commands.calibrate;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.system.motor.MotorSystem;
import botsnax.system.motor.MotorState;
import botsnax.system.Gearbox;

import static edu.wpi.first.units.Units.Degrees;

public record ArmCalibrationParams(
        Gearbox gearbox,
        Measure<Angle> range,
        Subsystem ... requirements
) {
    public MotorState getMotorState(Measure<Time> time) {
        return gearbox.getMotor().getState(time);
    }

    public MotorSystem encoderMotorSystem() {
        return gearbox.getMotor().withEncoder(gearbox.getOutputEncoder(), false);
    }

    public MotorState getEncoderState(Measure<Time> time) {
        return new MotorState(gearbox().getMotor().getVoltage(), time, gearbox.getOutputEncoder().getAngle(), gearbox.getOutputEncoder().getVelocity());
    }

    public void setVoltage(double voltage) {
        gearbox.getMotor().setVoltage(voltage);
    }

    public Measure<Angle> getAngle() {
        return gearbox.getOutputEncoder().getAngle();
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return gearbox.getOutputEncoder().getVelocity();
    }

    public void zeroPositions() {
        gearbox.getMotor().setAngle(Degrees.of(0));
        gearbox.getOutputEncoder().setAngle(Degrees.of(0));
    }
}
