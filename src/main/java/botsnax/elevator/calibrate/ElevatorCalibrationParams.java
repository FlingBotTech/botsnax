package botsnax.elevator.calibrate;

import botsnax.system.motor.MotorSystem;
import botsnax.util.LinearAngularConversion;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public record ElevatorCalibrationParams(
        MotorSystem motor,
        LinearAngularConversion linearAngularConversion,
        Distance height,
        Subsystem... requirements) {
}
