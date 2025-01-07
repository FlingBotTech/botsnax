package botsnax.system.motor.phoenix;

import botsnax.system.motor.MotorSystem;
import botsnax.util.phoenix.PhoenixUtil;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class TalonFXMotor implements MotorSystem {
    private final TalonFX motor;
    private final double invert;

    public TalonFXMotor(TalonFX motor, boolean inverted) {
        this.motor = motor;

        if (RobotBase.isSimulation()) {
            setInverted(inverted);
        }

        boolean motorInverted = getInverted();

        if (inverted != motorInverted) {
            System.err.println("WARNING: TalonFX " + motor.getDeviceID() + " inversion config does not match code. Emulating inversion.");
        }

        this.invert = (inverted != motorInverted) ? -1 : 1;
    }

    private void setInverted(boolean inverted) {
        MotorOutputConfigs configs = new MotorOutputConfigs();

        throwFail(motor.getConfigurator().refresh(configs), "get configuration");
        configs.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        throwFail(motor.getConfigurator().apply(configs), "set inverted");
    }

    private boolean getInverted() {
        MotorOutputConfigs configs = new MotorOutputConfigs();

        throwFail(motor.getConfigurator().refresh(configs), "get configuration");

        return configs.Inverted == InvertedValue.Clockwise_Positive;
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

    public TalonFXSimState getSimState() {
        return motor.getSimState();
    }

    @Override
    public Angle getAngle() {
        return motor.getRotorPosition().getValue().times(invert);
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getRotorVelocity().getValue().times(invert);
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue().times(invert);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.times(invert).baseUnitMagnitude());
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setAngle(Angle angle) {
        PhoenixUtil.setAndValidatePosition(motor, angle.times(invert));
    }
}
