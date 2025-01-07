package botsnax.swerve.commands.calibrate;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.swerve.SwerveModule;
import botsnax.system.motor.AngleSetter;

import static edu.wpi.first.units.Units.Volts;

public class RotateAtVoltageCommand extends Command {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final AngleSetter angleSetter;
    private final int moduleIndex;
    private final Voltage voltage;

    public RotateAtVoltageCommand(
            SwerveDriveKinematics kinematics,
            SwerveModule[] modules,
            AngleSetter angleSetter,
            int moduleIndex,
            Voltage voltage,
            Subsystem ... requirements) {
        this.kinematics = kinematics;
        this.modules = modules;
        this.angleSetter = angleSetter;
        this.moduleIndex = moduleIndex;
        this.voltage = voltage;

        addRequirements(requirements);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
        var states = kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modules.length; i++) {
            Voltage moduleVoltage = (i == moduleIndex) ? voltage : Volts.of(0);

            modules[i].getDrive().setVoltage(moduleVoltage);
            angleSetter.apply(states[i].angle, modules[i].getSteering().getMotor());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
