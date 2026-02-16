package botsnax.system.motor.phoenix;

import botsnax.control.periodic.ApplyMode;
import botsnax.system.motor.MotorSim;
import botsnax.system.motor.MotorSystem;
import botsnax.system.motor.VelocitySetter;
import botsnax.util.phoenix.PhoenixUtil;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class TalonFXMotor implements MotorSystem {
    private class Sim implements MotorSim {
        @Override
        public void setSupplyVoltage(Voltage voltage) {
            motor.getSimState().setSupplyVoltage(voltage.baseUnitMagnitude());
        }

        @Override
        public void setInverted(boolean isInverted) {
            motor.getSimState().Orientation = isInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        }

        @Override
        public Voltage getMotorVoltage() {
            return Volts.of(motor.getSimState().getMotorVoltage());
        }

        @Override
        public void setRawPosition(Angle angle) {
            motor.getSimState().setRawRotorPosition(angle.in(Rotations));
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            motor.getSimState().setRotorVelocity(velocity.in(RotationsPerSecond));
        }
    }

    private final TalonFX motor;
    private final Sim sim;
    private final DCMotor dcMotor;

    public TalonFXMotor(TalonFX motor) {
        this(motor, DCMotor.getKrakenX60(1));
    }

    public TalonFXMotor(TalonFX motor, DCMotor dcMotor) {
        this.motor = motor;

        if (isSimulation()) {
            sim = new Sim();
        } else {
            sim = null;
        }

        this.dcMotor = dcMotor;
    }

    @Override
    public DCMotor getDCMotor() {
        return dcMotor;
    }

    private void throwFail(StatusCode statusCode, String operation) {
        if (!statusCode.isOK()) {
            String message = "Unable to " + operation + " on TalonFX ID " + motor.getDeviceID() + ": " + statusCode;
            System.err.println(message);
            throw new RuntimeException(message);
        }
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

    public MotorSim getSim() {
        return sim;
    }

    @Override
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getRotorVelocity().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setAngle(Angle angle) {
        PhoenixUtil.setAndValidatePosition(motor, angle);
    }

    public static VelocitySetter getVelocityVoltageSetter() {
        final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

        return (velocity, flywheel) -> {
            ((TalonFXMotor) flywheel).get().setControl(velocityVoltage.withVelocity(velocity));
        };
    }

    public static ApplyMode<Angle, MotorSystem> applyModeOf(final MotionMagicVoltage motionMagicVoltage) {
        return (angle, motor) -> {
            motionMagicVoltage.Position = angle.in(Rotations);
            ((TalonFXMotor) motor).get().setControl(motionMagicVoltage);
        };
    }

    public static ApplyMode<Angle, MotorSystem> applyModeOf(final PositionVoltage positionVoltage) {
        return (angle, motor) -> {
            positionVoltage.Position = angle.in(Rotations);
            ((TalonFXMotor) motor).get().setControl(positionVoltage);
        };
    }
}
