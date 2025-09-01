package botsnax.swerve.commands.calibrate;

import botsnax.swerve.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Rotations;

public class StartOdometryCalibrationCommand extends Command {
    private SwerveModule[] modules;

    public StartOdometryCalibrationCommand(SwerveModule[] modules, Subsystem... requirements) {
        this.modules = modules;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        for (SwerveModule module : modules) {
            module.getDrive().setCoastOnIdle();
            module.getDrive().setAngle(Rotations.zero());
        }
    }
}
