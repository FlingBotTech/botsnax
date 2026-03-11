package botsnax.turret.calibrate;

import botsnax.commands.calibrate.VelocityCalibration;
import botsnax.commands.calibrate.VelocityCalibrationData;
import botsnax.system.Gyro;
import botsnax.system.motor.MotorSystem;
import botsnax.util.AngularVelocityData;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static java.lang.Math.signum;

public class CalibrateTurretCommand extends SequentialCommandGroup {
    private final MotorSystem motor;
    private final Gyro gyro;
    private final Rotation2d maxOffset;
    private final Subsystem subsystem;

    private Rotation2d center = null;

    public CalibrateTurretCommand(MotorSystem motor, Gyro gyro, Angle range, Subsystem subsystem) {
        this.motor = motor;
        this.gyro = gyro;
        this.subsystem = subsystem;
        this.maxOffset = Rotation2d.fromRadians(range.div(2).in(Radians));

        List<Voltage> voltages = List.of(
                Volts.of(1),
                Volts.of(2),
                Volts.of(3),
                Volts.of(4)
        );

        addCommands(voltages);
    }

    private void addCommands(List<Voltage> voltages) {
        VelocityCalibrationData velocityCalibrationData = new VelocityCalibrationData();

        addCommands(
                init(),
                home(voltages)
        );

        for (Voltage voltage : voltages) {
            addCommands(
                    determineAverageVelocity(voltage, velocityCalibrationData),
                    home(voltages)
            );
        }

        addCommands(
                new InstantCommand(() -> {
                    VelocityCalibration calibration = velocityCalibrationData.solve();
                    calibration.save(TurretCalibration.PREFERENCE_BASE_NAME);
                })
        );
    }

    private Command home(List<Voltage> voltages) {
        return moveToLimit(voltages.get(0).times(-1), v -> {})
                .andThen(new WaitCommand(1));
    }

    private boolean isOutOfRange(Angle angle, double direction) {
        Rotation2d oriented = Rotation2d.fromRadians(angle.in(Radians)).minus(center);

        return ((signum(oriented.getSin()) == signum(direction)) && (oriented.getCos() < maxOffset.getCos()));
    }

    private Command init() {
        return subsystem.runOnce(() -> {
            center = Rotation2d.fromRadians(motor.getAngle().in(Radians));
        });
    }

    private Command setVoltage(Voltage voltage) {
        return subsystem.runOnce(() -> motor.setVoltage(voltage));
    }

    private Command determineAverageVelocity(Voltage voltage, VelocityCalibrationData velocityCalibrationData) {
        final AngularVelocityData data = new AngularVelocityData();

        return moveToLimit(voltage, data :: add)
                .andThen(new InstantCommand(() -> {
                    AngularVelocity averageVelocity = data.summarize();
                    System.out.println("Average velocity at " + voltage + " => " + averageVelocity);
                    velocityCalibrationData.add(averageVelocity, voltage);
                }));
    }

    private Command moveToLimit(Voltage voltage, Consumer<AngularVelocity> velocityConsumer) {
        return setVoltage(voltage)
                .andThen(subsystem
                        .run(() -> velocityConsumer.accept(gyro.getVelocity()))
                        .until(() -> isOutOfRange(motor.getAngle(), voltage.baseUnitMagnitude())))
                .andThen(setVoltage(Volts.zero()));
    }
}
