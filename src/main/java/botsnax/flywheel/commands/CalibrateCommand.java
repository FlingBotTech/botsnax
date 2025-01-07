package botsnax.flywheel.commands;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.commands.calibrate.AwaitStableValueCommand;
import botsnax.commands.calibrate.VelocityCalibration;
import botsnax.commands.calibrate.VelocityCalibrationData;
import botsnax.flywheel.Flywheel;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class CalibrateCommand extends SequentialCommandGroup {
    private static final int NUM_INCREMENTS = 8;

    private final Flywheel flywheel;
    private final VelocityCalibrationData data = new VelocityCalibrationData();
    private final Subsystem subsystem;

    public CalibrateCommand(Flywheel flywheel, Voltage maxVoltage, Consumer<VelocityCalibration> consumer, Subsystem subsystem) {
        this.flywheel = flywheel;
        this.subsystem = subsystem;

        for (int i = 1; i <= NUM_INCREMENTS; i++) {
            double percentage = (double) i / (NUM_INCREMENTS + 1);
            addCommands(testVoltageCommand(maxVoltage.times(percentage)));
        }

        addCommands(subsystem.runOnce(flywheel::stop));
        addCommands(new InstantCommand(() -> consumer.accept(data.solve())));
    }

    private Command testVoltageCommand(Voltage voltage) {
        return subsystem
                .runOnce(() -> flywheel.setVoltage(voltage))
                .andThen(new AwaitStableValueCommand(
                                Seconds.of(2),
                                RadiansPerSecond.of(0.05).baseUnitMagnitude(),
                                () -> flywheel.getVelocity().baseUnitMagnitude(),
                                value -> data.add(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(value), voltage),
                                subsystem
                        ));
    }
}
