package botsnax.system.motor.phoenix;

import botsnax.system.motor.MotorSim;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class TalonFXSMotor extends TalonMotor<TalonFXS> {
    private class Sim implements MotorSim {
        @Override
        public void setSupplyVoltage(Voltage voltage) {
            motor.getSimState().setSupplyVoltage(voltage.baseUnitMagnitude());
        }

        @Override
        public void setInverted(boolean isInverted) {
            motor.getSimState().MotorOrientation = isInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
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

    public TalonFXSMotor(TalonFXS motor, DCMotor dcMotor) {
        super(motor, dcMotor);
    }

    @Override
    protected MotorSim createSim() {
        return new Sim();
    }

    @Override
    public void setCoastOnIdle() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void setBrakeOnIdle() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }
}
