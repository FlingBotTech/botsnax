package botsnax.util;

import edu.wpi.first.math.system.plant.DCMotor;

public class DCMotorUtil {
    public static DCMotor withMotorCount(DCMotor motor, int count) {
        return new DCMotor(
                motor.nominalVoltageVolts,
                motor.stallTorqueNewtonMeters,
                motor.stallCurrentAmps,
                motor.freeCurrentAmps,
                motor.freeSpeedRadPerSec,
                count
        );
    }
}
