package botsnax.system.motor.phoenix;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import botsnax.system.Gearbox;
import botsnax.system.encoder.phoenix.TalonSRXAbsoluteEncoder;
import botsnax.system.motor.AngleSetter;
import botsnax.system.motor.MotorSystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

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
    public void setVoltage(Voltage voltage) {
        motor.set(TalonSRXControlMode.PercentOutput, Math.min(1.0, 12.0 / motor.getBusVoltage() * voltage.baseUnitMagnitude()));
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getMotorOutputVoltage());
    }

    @Override
    public void stop() {
        motor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    @Override
    public Angle getAngle() {
        return Rotations.of(motor.getSelectedSensorPosition(PRIMARY_CLOSED_LOOP_INDEX) / ENCODER_RESOLUTION);
    }

    public double getAbsolutePosition() {
        return motor.getSensorCollection().getPulseWidthPosition();
    }

    public Angle getOffset() {
        return Rotations.of(offset / ENCODER_RESOLUTION);
    }

    public Angle getAbsoluteAngle() {
        return Rotations.of((getAbsolutePosition() - offset) / ENCODER_RESOLUTION);
    }

    public void setAbsoluteAngle(Angle angle) {
        offset = getAbsolutePosition() - (angle.in(Rotations) * ENCODER_RESOLUTION);
    }

    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(motor.getSelectedSensorVelocity(PRIMARY_CLOSED_LOOP_INDEX) / ENCODER_RESOLUTION * VELOCITY_TIME_UNITS_PER_SECOND);
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setSelectedSensorPosition(angle.in(Rotations) * ENCODER_RESOLUTION, PRIMARY_CLOSED_LOOP_INDEX, 0);
    }

    public TalonSRX getMotor() {
        return motor;
    }

    public Gearbox asGearbox() {
        return new Gearbox(this, new TalonSRXAbsoluteEncoder(this));
    }
}
