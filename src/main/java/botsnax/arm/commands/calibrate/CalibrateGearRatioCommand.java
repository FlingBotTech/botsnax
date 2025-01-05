package botsnax.arm.commands.calibrate;

import botsnax.arm.commands.AwaitStabilityCommand;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import botsnax.control.MotorController;
import botsnax.control.MotorControllerFactory;
import botsnax.control.ProportionalSetpointController;
import botsnax.commands.calibrate.VelocityCalibration;

import java.util.function.Consumer;

import static edu.wpi.first.units.ImmutableMeasure.ofRelativeUnits;
import static edu.wpi.first.units.Units.*;
import static java.lang.Math.pow;

public class CalibrateGearRatioCommand extends SequentialCommandGroup {
    private static final Measure<Angle> ERROR_TO_APPLY_MAXIMUM_VOLTAGE = ofRelativeUnits(10, Degrees);

    public CalibrateGearRatioCommand(ArmCalibrationParams arm, double maxVoltage, Consumer<ArmCalibration> calibrationConsumer) {
        addCommands(
                new InstantCommand(arm::zeroPositions, arm.requirements()),
                goToAngle(arm, ERROR_TO_APPLY_MAXIMUM_VOLTAGE, maxVoltage, calibrationConsumer)
        );
    }

    private Command goToAngle(ArmCalibrationParams arm, Measure<Angle> endAngle, double maxVoltage, Consumer<ArmCalibration> calibrationConsumer) {
        MotorController controller = createSCurveController(Degrees.of(0), endAngle, maxVoltage);
        return new AwaitStabilityCommand(arm, controller, result -> {
            double gearRatio = result.motorAngle().in(Radians) / result.angle().in(Radians);
            calibrationConsumer.accept(createEstimatedCalibration(result, gearRatio));
        });
    }

    private ArmCalibration createEstimatedCalibration(AwaitStabilityCommand.Result result, double gearRatio) {
        DCMotor motor = DCMotor.getKrakenX60(1);
        double estimatedVelocityToVoltageSlope = pow(motor.freeSpeedRadPerSec / motor.nominalVoltageVolts / gearRatio, -1);

        return new ArmCalibration(
                new ArmGravityController(result.voltage(), result.angle()),
                gearRatio,
                new VelocityCalibration(estimatedVelocityToVoltageSlope, 0),
                new VelocityCalibration(estimatedVelocityToVoltageSlope, 0)
        );
    }

    private MotorController createSCurveController(Measure<Angle> startAngle, Measure<Angle> endAngle,double maxVoltage) {
        return MotorControllerFactory.createSCurveController(
                startAngle,
                endAngle,
                ofRelativeUnits(5, DegreesPerSecond),
                ProportionalSetpointController.create(maxVoltage, ERROR_TO_APPLY_MAXIMUM_VOLTAGE)
        );
    }
}
