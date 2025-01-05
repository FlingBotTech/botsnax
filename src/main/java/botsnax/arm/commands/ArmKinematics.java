package botsnax.arm.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.control.MotorKinematics;
import botsnax.system.motor.MotorState;

public interface ArmKinematics extends MotorKinematics {
    double getGearRatio();
    Measure<Angle> getHorizontalAngle();

    public static class Stub implements ArmKinematics {
        private final double gearRatio;
        private final Measure<Angle> horizontalAngle;

        public Stub(double gearRatio, Measure<Angle> horizontalAngle) {
            this.gearRatio = gearRatio;
            this.horizontalAngle = horizontalAngle;
        }

        @Override
        public double getGearRatio() {
            return gearRatio;
        }

        @Override
        public Measure<Angle> getHorizontalAngle() {
            return horizontalAngle;
        }

        @Override
        public double getVoltageForVelocity(Measure<Velocity<Angle>> velocity, MotorState state) {
            return 0;
        }
    }
}
