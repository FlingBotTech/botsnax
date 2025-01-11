package botsnax.system.motor;

import botsnax.flywheel.Flywheel;
import botsnax.system.encoder.Encoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface MotorSystem extends Flywheel {
    void stop();

    default MotorState getState(Time time) {
        return new MotorState(getVoltage(), time, getAngle(), getVelocity() );
    }

    default MotorSystem withGearRatio(double gearRatio) {
        return new GearedMotorSystem(this, gearRatio);
    }

    MotorSim getSim();

    default MotorSystem withEncoder(Encoder encoder, boolean allowSetAngle) {
        return new MotorSystem() {
            @Override
            public int getDeviceID() {
                return MotorSystem.this.getDeviceID();
            }

            @Override
            public MotorSim getSim() {
                return MotorSystem.this.getSim();
            }

            @Override
            public DCMotor getDCMotor() {
                return MotorSystem.this.getDCMotor();
            }

            @Override
            public void setBrakeOnIdle() {
                MotorSystem.this.setBrakeOnIdle();
            }

            @Override
            public void setCoastOnIdle() {
                MotorSystem.this.setCoastOnIdle();
            }

            @Override
            public Angle getAngle() {
                return encoder.getAngle();
            }

            @Override
            public AngularVelocity getVelocity() {
                return encoder.getVelocity();
            }

            @Override
            public Voltage getVoltage() {
                return MotorSystem.this.getVoltage();
            }

            @Override
            public void setVoltage(Voltage voltage) {
                MotorSystem.this.setVoltage(voltage);
            }

            @Override
            public void stop() {
                MotorSystem.this.stop();
            }

            @Override
            public void setAngle(Angle angle) {
                if (allowSetAngle) {
                    encoder.setAngle(angle);
                }
            }
        };
    }

    MotorSystem STUB = new MotorSystem() {
        @Override
        public int getDeviceID() {
            return -1;
        }

        @Override
        public MotorSim getSim() {
            return MotorSim.STUB;
        }

        @Override
        public DCMotor getDCMotor() {
            return DCMotor.getKrakenX60(1);
        }

        @Override
        public void stop() {
        }

        @Override
        public void setBrakeOnIdle() {
        }

        @Override
        public void setCoastOnIdle() {
        }

        @Override
        public Angle getAngle() {
            return Degrees.of(0);
        }

        @Override
        public AngularVelocity getVelocity() {
            return Degrees.of(0).per(Second);
        }

        @Override
        public void setAngle(Angle angle) {
        }

        @Override
        public void setVoltage(Voltage voltage) {
        }

        @Override
        public Voltage getVoltage() {
            return Volts.of(0);
        }
    };
}
