package botsnax.swerve.commands.calibrate;

import botsnax.swerve.SwerveCalibration;
import botsnax.swerve.SwerveModule;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Consumer;

import static botsnax.swerve.SwerveCalibration.getCouplingRatioName;
import static java.lang.Math.abs;

public class CalibrateCouplingRatioCommand extends SequentialCommandGroup {
    public record Result(Angle steeringAngleDelta, Angle driveAngleDelta) {
        public double getRatio() {
            return driveAngleDelta.baseUnitMagnitude() / steeringAngleDelta.baseUnitMagnitude();
        }

        public void store(int index) {
            final String preferenceName = getCouplingRatioName(index);
            Preferences.setDouble(preferenceName, getRatio());
        }
    }

    public CalibrateCouplingRatioCommand(SwerveModule[] modules, Angle steeringAngleDelta, Voltage steeringVoltage, Subsystem ... requirements) {
        for (int i = 0; i < modules.length; i++) {
            final int index = i;
            SwerveModule module = modules[i];

            addCommands(create(module, steeringAngleDelta, steeringVoltage, result -> {
                System.out.println("Coupling ratio " + index + " = " + result.getRatio());
                result.store(index);
            } ));
        }
    }

    public static Command create(SwerveModule module, Angle steeringAngleDelta, Voltage steeringVoltage, Consumer<Result> consumer, Subsystem... requirements) {
        final Angle initialSteeringAngle = module.getSteering().getOutputEncoder().getAngle();
        final Angle initialDriveAngle = module.getDrive().getAngle();

        return new InstantCommand(() -> {
            module.getDrive().setBrakeOnIdle();
            module.getSteering().getMotor().setVoltage(steeringVoltage);
        }, requirements)
                .andThen(new WaitUntilCommand(() -> abs(module.getSteering().getMotor().getAngle().minus(initialSteeringAngle).baseUnitMagnitude()) > steeringAngleDelta.baseUnitMagnitude()))
                .andThen(new InstantCommand(() -> module.getSteering().getMotor().stop(), requirements))
                .andThen(new WaitCommand(1))
                .andThen(new InstantCommand(() -> consumer.accept(
                        new Result(
                                module.getSteering().getMotor().getAngle().minus(initialSteeringAngle),
                                module.getDrive().getAngle().minus(initialDriveAngle))),
                        requirements))
                ;
    }
}
