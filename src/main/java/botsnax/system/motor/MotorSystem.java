package botsnax.system.motor;

import botsnax.system.encoder.Encoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import botsnax.flywheel.Flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;

public interface MotorSystem extends Flywheel {
    void stop();

    default MotorState getState(Measure<Time> time) {
        return new MotorState(getVoltage(), time, getAngle(), getVelocity() );
    }

    default MotorSystem withGearRatio(double gearRatio) {
        return new GearedMotorSystem(this, gearRatio);
    }

    default MotorSystem withEncoder(Encoder encoder, boolean allowSetAngle) {
        return new MotorSystem() {
            @Override
            public int getDeviceID() {
                return MotorSystem.this.getDeviceID();
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
            public Measure<Angle> getAngle() {
                return encoder.getAngle();
            }

            @Override
            public Measure<Velocity<Angle>> getVelocity() {
                return encoder.getVelocity();
            }

            @Override
            public double getVoltage() {
                return MotorSystem.this.getVoltage();
            }

            @Override
            public void setVoltage(double voltage) {
                MotorSystem.this.setVoltage(voltage);
            }

            @Override
            public void stop() {
                MotorSystem.this.stop();
            }

            @Override
            public void setAngle(Measure<Angle> angle) {
                if (allowSetAngle) {
                    encoder.setAngle(angle);
                }
            }
        };
    }

    public static final MotorSystem STUB = new MotorSystem() {
        @Override
        public int getDeviceID() {
            return -1;
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
        public Measure<Angle> getAngle() {
            return Degrees.of(0);
        }

        @Override
        public Measure<Velocity<Angle>> getVelocity() {
            return Degrees.of(0).per(Second);
        }

        @Override
        public void setAngle(Measure<Angle> angle) {
        }

        @Override
        public void setVoltage(double voltage) {
        }

        @Override
        public double getVoltage() {
            return 0;
        }
    };
}
