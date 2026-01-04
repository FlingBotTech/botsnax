package botsnax.system.motor.phoenix;

import botsnax.control.periodic.ApplyMode;
import botsnax.swerve.sim.PerfectSteering;
import botsnax.system.motor.MotorSim;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import botsnax.system.Gearbox;
import botsnax.system.encoder.phoenix.TalonSRXAbsoluteEncoder;
import botsnax.system.motor.AngleSetter;
import botsnax.system.motor.MotorSystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static java.lang.Math.round;

public class TalonSRXMotor implements MotorSystem {
    public class Sim implements MotorSim {
        @Override
        public void setSupplyVoltage(Voltage voltage) {
            motor.getSimCollection().setBusVoltage(voltage.in(Volts));
        }

        @Override
        public void setInverted(boolean isInverted) {
        }

        @Override
        public Voltage getMotorVoltage() {
            return Volts.of(motor.getSimCollection().getMotorOutputLeadVoltage());
        }

        @Override
        public void setRawPosition(Angle angle) {
            motor.getSimCollection().setQuadratureRawPosition((int) round(angle.in(Rotations) * ENCODER_RESOLUTION));
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            motor.getSimCollection().setQuadratureVelocity((int) round(velocity.in(RotationsPerSecond) * ENCODER_RESOLUTION / VELOCITY_TIME_UNITS_PER_SECOND));
        }
    }

    public static final double ENCODER_RESOLUTION = 4096.0;
    public static final double VELOCITY_TIME_UNITS_PER_SECOND = 10;
    private static final int PRIMARY_CLOSED_LOOP_INDEX  = 0;

    public static AngleSetter PID_CONTROL = (angle, motor) -> {
        if (motor instanceof TalonSRXMotor talonSRX) {
            double currentAngle = talonSRX.get().getSelectedSensorPosition() / ENCODER_RESOLUTION;
            double errorRotations = currentAngle - angle.getRotations();
            double fullRotations = round(errorRotations);
            double setpointRotations = angle.getRotations() + fullRotations;

            talonSRX.get().set(TalonSRXControlMode.Position, setpointRotations * ENCODER_RESOLUTION);
        } else if (motor instanceof PerfectSteering perfectSteering) {
            perfectSteering.setAngle(Radians.of(angle.getRadians()));
        }
    };

    public static ApplyMode<Angle, MotorSystem> PID_APPLY_MODE = (angle, motor) ->
            PID_CONTROL.apply(new Rotation2d(angle), motor);

    private final TalonSRX motor;
    private final DCMotor dcMotor;
    private final Sim sim;
    private double offset = 0;

    public TalonSRXMotor(TalonSRX motor, DCMotor dcMotor) {
        this.motor = motor;
        this.sim = isSimulation() ? new Sim() : null;
        this.dcMotor = dcMotor;
    }

    public TalonSRX get() {
        return motor;
    }

    @Override
    public DCMotor getDCMotor() {
        return dcMotor;
    }

    public MotorSim getSim() {
        return sim;
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
