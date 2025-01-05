package botsnax.swerve.commands.calibrate;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.swerve.SwerveModule;
import botsnax.system.encoder.AbsoluteEncoder;

import static botsnax.swerve.SwerveCalibration.getOffsetName;

public class StoreEncoderOffsetsCommand extends SequentialCommandGroup {
    public StoreEncoderOffsetsCommand(SwerveModule[] modules, Subsystem... requirements) {
        for (int i = 0; i < modules.length; i++) {
            final String preferenceName = getOffsetName(i);
            final AbsoluteEncoder encoder = modules[i].getSteering().getOutputEncoder();

            addCommands(
                    new InstantCommand(() -> {
                        Preferences.setDouble(preferenceName, encoder.getZeroingValue());
                    }, requirements)
            );
        }
    }
}
