package botsnax.elevator.calibrate;

import botsnax.commands.calibrate.DirectionalVelocityCalibration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import java.util.Optional;

import static edu.wpi.first.units.Units.Volts;

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

    public Slot0Configs getTalonFXSlot0Configs() {
        return velocityCalibration.positiveVelocityCalibration().getTalonFXSlot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKG(gravityController.gravityVoltage().in(Volts))
                ;
    }
}
