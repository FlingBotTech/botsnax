package botsnax.elevator.calibrate;

import botsnax.commands.calibrate.DirectionalVelocityCalibration;

import java.util.Optional;

public record ElevatorCalibration (
        ElevatorGravityController gravityController,
        DirectionalVelocityCalibration velocityCalibration
) {
    public void save(String baseName) {
        gravityController.save(baseName);
        velocityCalibration.save(baseName);
    }

    public static Optional<ElevatorCalibration> load(String baseName) {
        return ElevatorGravityController.load(baseName).flatMap(gravityController ->
                DirectionalVelocityCalibration.load(baseName).map(velocityCalibration ->
                        new ElevatorCalibration(gravityController, velocityCalibration)));
    }
}
