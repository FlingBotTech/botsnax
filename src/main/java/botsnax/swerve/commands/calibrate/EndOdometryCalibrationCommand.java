package botsnax.swerve.commands.calibrate;

import botsnax.swerve.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Rotations;

public class EndOdometryCalibrationCommand extends Command {
    private SwerveModule[] modules;

    public EndOdometryCalibrationCommand(SwerveModule[] modules, Subsystem... requirements) {
        this.modules = modules;
        addRequirements(requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        for (int i = 0; i < modules.length; i++) {
            System.out.println("Module " + i + ": " + modules[i].getDrive().getAngle().in(Rotations));
        }
    }
}
