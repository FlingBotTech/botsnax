package botsnax.commands.calibrate;

import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import java.util.Optional;

import static edu.wpi.first.units.Units.Volts;
import static java.lang.Math.signum;

public record DirectionalVelocityCalibration(
        VelocityCalibration positiveVelocityCalibration,
        VelocityCalibration negativeVelocityCalibration
) {

    public Voltage getVoltage(AngularVelocity velocity, MotorState state) {
        double signOfVelocity = signum(velocity.baseUnitMagnitude());

        if (signOfVelocity > 0) {
            return positiveVelocityCalibration.getVoltageForVelocity(velocity, state);
        } else if (signOfVelocity < 0) {
            return negativeVelocityCalibration.getVoltageForVelocity(velocity, state);
        } else {
            return Volts.of(0);
        }
    }

    public void save(String baseName) {
        positiveVelocityCalibration.save(baseName + ".positive");
        negativeVelocityCalibration.save(baseName + ".negative");
    }

    public static Optional<DirectionalVelocityCalibration> load(String baseName) {
            return VelocityCalibration.load(baseName + ".positive").flatMap(positiveVelocityCalibration ->
                            VelocityCalibration.load(baseName + ".negative").map(negativeVelocityCalibration ->
                                    new DirectionalVelocityCalibration(positiveVelocityCalibration, negativeVelocityCalibration)));
    }
}
