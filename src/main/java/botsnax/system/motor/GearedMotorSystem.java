package botsnax.system.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class GearedMotorSystem implements MotorSystem {
    private class Sim implements MotorSim {
        @Override
        public Voltage getMotorVoltage() {
            return motor.getSim().getMotorVoltage();
        }

        @Override
        public void setSupplyVoltage(Voltage voltage) {
            motor.getSim().setSupplyVoltage(voltage);
        }

        @Override
        public void setInverted(boolean isInverted) {
            motor.getSim().setInverted(isInverted);
        }

        @Override
        public void setRawPosition(Angle angle) {
            motor.getSim().setRawPosition(angle.times(gearRatio));
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            motor.getSim().setVelocity(velocity.times(gearRatio));
        }
    }

    private final MotorSystem motor;
    private final double gearRatio;
    private final Sim sim;
    private final DCMotor dcMotor;

    public GearedMotorSystem(MotorSystem motor, double gearRatio) {
        this.motor = motor;
        this.gearRatio = gearRatio;
        this.sim = isSimulation() ? new Sim() : null;

        DCMotor baseDCMotor = motor.getDCMotor();
        this.dcMotor = new DCMotor(
                baseDCMotor.nominalVoltageVolts,
                baseDCMotor.stallTorqueNewtonMeters * gearRatio,
                baseDCMotor.stallCurrentAmps * gearRatio,
                baseDCMotor.freeCurrentAmps,
                baseDCMotor.freeSpeedRadPerSec / gearRatio,
                1
        );
    }

    @Override
    public MotorSim getSim() {
        return sim;
    }

    @Override
    public DCMotor getDCMotor() {
        return dcMotor;
    }

    @Override
    public int getDeviceID() {
        return motor.getDeviceID();
    }

    @Override
    public void setBrakeOnIdle() {
        motor.setBrakeOnIdle();
    }

    @Override
    public void setCoastOnIdle() {
        motor.setBrakeOnIdle();
    }

    @Override
    public Angle getAngle() {
        return motor.getAngle().div(gearRatio);
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().div(gearRatio);
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setAngle(angle.times(gearRatio));
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public Voltage getVoltage() {
        return motor.getVoltage();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }
}
