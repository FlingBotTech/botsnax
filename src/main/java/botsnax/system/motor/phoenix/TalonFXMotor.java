package botsnax.system.motor.phoenix;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import botsnax.system.motor.MotorSystem;
import botsnax.util.phoenix.PhoenixUtil;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

public class TalonFXMotor implements MotorSystem {
    private static final Unit<Angle> ANGLE_UNITS = Rotations;
    private static final Unit<Velocity<Angle>> VELOCITY_UNITS = ANGLE_UNITS.per(Second);

    private final TalonFX motor;
    private final double invert;

    public TalonFXMotor(TalonFX motor, boolean inverted) {
        this.motor = motor;

        if (RobotBase.isSimulation()) {
            motor.setInverted(inverted);
        }

        boolean motorInverted = motor.getInverted();

        if (inverted != motorInverted) {
            System.err.println("WARNING: TalonFX " + motor.getDeviceID() + " inversion config does not match code. Emulating inversion.");
        }

        this.invert = (inverted != motorInverted) ? -1 : 1;
    }

    public TalonFX get() {
        return motor;
    }

    @Override
    public int getDeviceID() {
        return motor.getDeviceID();
    }

    @Override
    public void setCoastOnIdle() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void setBrakeOnIdle() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public TalonFXSimState getSimState() {
        return motor.getSimState();
    }

    @Override
    public Measure<Angle> getAngle() {
        return ANGLE_UNITS.of(motor.getRotorPosition().getValue()).times(invert);
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return VELOCITY_UNITS.of(motor.getRotorVelocity().getValue()).times(invert);
    }

    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValue() * invert;
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage * invert);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setAngle(Measure<Angle> angle) {
        PhoenixUtil.setAndValidatePosition(motor, angle.times(invert));
    }
}
