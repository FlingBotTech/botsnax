package botsnax.arm.commands;

import botsnax.control.MotorKinematics;
import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public interface ArmKinematics extends MotorKinematics {
    double getGearRatio();
    Angle getHorizontalAngle();

    public static class Stub implements ArmKinematics {
        private final double gearRatio;
        private final Angle horizontalAngle;

        public Stub(double gearRatio, Angle horizontalAngle) {
            this.gearRatio = gearRatio;
            this.horizontalAngle = horizontalAngle;
        }

        @Override
        public double getGearRatio() {
            return gearRatio;
        }

        @Override
        public Angle getHorizontalAngle() {
            return horizontalAngle;
        }

        @Override
        public Voltage getVoltageForVelocity(AngularVelocity velocity, MotorState state) {
            return Volts.of(0);
        }
    }
}
