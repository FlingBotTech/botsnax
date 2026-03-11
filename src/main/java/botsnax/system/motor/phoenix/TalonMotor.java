package botsnax.system.motor.phoenix;

import botsnax.control.periodic.ApplyMode;
import botsnax.system.motor.MotorSim;
import botsnax.system.motor.MotorSystem;
import botsnax.system.motor.VelocitySetter;
import botsnax.util.phoenix.PhoenixUtil;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public abstract class TalonMotor<T extends CommonTalon> implements MotorSystem {
    protected final T motor;
    private final MotorSim sim;
    private final DCMotor dcMotor;

    private final VoltageOut voltageOut;
    private final NeutralOut neutralOut;

    public TalonMotor(T motor, DCMotor dcMotor) {
        this.motor = motor;

        if (isSimulation()) {
            sim = createSim();
        } else {
            sim = null;
        }

        this.dcMotor = dcMotor;
        this.voltageOut = new VoltageOut(Volts.zero());
        this.neutralOut = new NeutralOut();
    }

    protected abstract MotorSim createSim();

    @Override
    public DCMotor getDCMotor() {
        return dcMotor;
    }

    protected void throwFail(StatusCode statusCode, String operation) {
        if (!statusCode.isOK()) {
            String message = "Unable to " + operation + " on device ID " + motor.getDeviceID() + ": " + statusCode;
            System.err.println(message);
            throw new RuntimeException(message);
        }
    }

    public T get() {
        return motor;
    }

    @Override
    public int getDeviceID() {
        return motor.getDeviceID();
    }

    @Override
    public abstract void setCoastOnIdle();

    @Override
    public abstract void setBrakeOnIdle();

    public MotorSim getSim() {
        return sim;
    }

    @Override
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stop() {
        motor.setControl(neutralOut);
    }

    @Override
    public void setAngle(Angle angle) {
        PhoenixUtil.setAndValidatePosition(motor, angle);
    }
}
