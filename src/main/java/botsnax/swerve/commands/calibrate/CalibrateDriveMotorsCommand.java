package botsnax.swerve.commands.calibrate;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import botsnax.swerve.SwerveCalibration;
import botsnax.swerve.SwerveModule;
import botsnax.system.motor.AngleSetter;

public class CalibrateDriveMotorsCommand extends SequentialCommandGroup {
    public CalibrateDriveMotorsCommand(
            SwerveDriveKinematics kinematics,
            SwerveModule[] modules,
            AngleSetter angleSetter,
            Subsystem ... requirements) {

        addCommands(
                new InstantCommand(() -> {
                    for (SwerveModule module : modules) {
                        module.getDrive().setCoastOnIdle();
                    }
                },requirements),
                new RotateAtVoltageCommand(
                        kinematics,
                        modules,
                        angleSetter,
                        0,
                        0,
                        requirements
                )
                        .alongWith(new WaitCommand(5))
        );

        for (int i = 0; i < modules.length; i++) {
            final String baseName = SwerveCalibration.getDriveVelocityBaseName(i);
            addCommands(
                    new CalibrateDriveMotorCommand(
                            kinematics,
                            modules,
                            i,
                            angleSetter,
                            calibration -> calibration.save(baseName),
                            requirements));
        }

        addCommands(new InstantCommand(() -> {
            for (SwerveModule module : modules) {
                module.getDrive().setBrakeOnIdle();
            }
        }, requirements));
    }
}
