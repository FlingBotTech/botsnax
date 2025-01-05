package botsnax.system.motor.phoenix;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.system.Gearbox;
import botsnax.system.encoder.phoenix.TalonSRXAbsoluteEncoder;
import botsnax.system.motor.AngleSetter;
import botsnax.system.motor.MotorSystem;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class TalonSRXMotor implements MotorSystem {
    private static final double ENCODER_RESOLUTION = 4096.0;
    private static final int PRIMARY_CLOSED_LOOP_INDEX  = 0;
    private static final double VELOCITY_TIME_UNITS_PER_SECOND = 10;

    public static AngleSetter PID_CONTROL = (angle, motor) -> {
        TalonSRX talonSRX = ((TalonSRXMotor) motor).getMotor();
        double currentAngle = talonSRX.getSelectedSensorPosition() / ENCODER_RESOLUTION;
        double errorRotations =  currentAngle - angle.getRotations();
        double fullRotations = Math.round(errorRotations);
        double setpointRotations = angle.getRotations() + fullRotations;

        talonSRX.set(TalonSRXControlMode.Position, setpointRotations * ENCODER_RESOLUTION);
    };

    private final TalonSRX motor;
    private double offset = 0;

    public TalonSRXMotor(TalonSRX motor) {
        this.motor = motor;
    }

    public TalonSRX get() {
        return motor;
    }

    @Override
    public int getDeviceID() {
        return motor.getDeviceID();
    }

    @Override
    public void setCoastOnIdle() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setBrakeOnIdle() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.set(TalonSRXControlMode.PercentOutput, Math.min(1.0, 12.0 / motor.getBusVoltage() * voltage));
    }

    @Override
    public double getVoltage() {
        return motor.getMotorOutputVoltage();
    }

    @Override
    public void stop() {
        motor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    @Override
    public Measure<Angle> getAngle() {
        return Rotations.of(motor.getSelectedSensorPosition(PRIMARY_CLOSED_LOOP_INDEX) / ENCODER_RESOLUTION);
    }

    public double getAbsolutePosition() {
        return motor.getSensorCollection().getPulseWidthPosition();
    }

    public Measure<Angle> getOffset() {
        return Rotations.of(offset / ENCODER_RESOLUTION);
    }

    public Measure<Angle> getAbsoluteAngle() {
        return Rotations.of((getAbsolutePosition() - offset) / ENCODER_RESOLUTION);
    }

    public void setAbsoluteAngle(Measure<Angle> angle) {
        offset = getAbsolutePosition() - (angle.in(Rotations) * ENCODER_RESOLUTION);
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return RotationsPerSecond.of(motor.getSelectedSensorVelocity(PRIMARY_CLOSED_LOOP_INDEX) / ENCODER_RESOLUTION * VELOCITY_TIME_UNITS_PER_SECOND);
    }

    @Override
    public void setAngle(Measure<Angle> angle) {
        motor.setSelectedSensorPosition(angle.in(Rotations) * ENCODER_RESOLUTION, PRIMARY_CLOSED_LOOP_INDEX, 0);
    }

    public TalonSRX getMotor() {
        return motor;
    }

    public Gearbox asGearbox() {
        return new Gearbox(this, new TalonSRXAbsoluteEncoder(this));
    }
}
