package botsnax.swerve;

import botsnax.swerve.SwerveModule.ApplyMode;
import botsnax.swerve.commands.calibrate.*;
import botsnax.system.motor.AngleSetter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class GenericSwerveDrivetrain {
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;

    public GenericSwerveDrivetrain(SwerveDriveKinematics kinematics, SwerveModule[] modules) {
        this.kinematics = kinematics;
        this.modules = modules;
    }

    public void apply(ChassisSpeeds speeds, ApplyMode mode) {
        apply(kinematics.toSwerveModuleStates(speeds), mode);
    }

    public void apply(SwerveModuleState[] states, ApplyMode mode) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].apply(states[i], mode);
        }
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Command calibrateDriveMotors(AngleSetter angleSetter, Subsystem... requirements) {
        return new CalibrateDriveMotorsCommand(kinematics, modules, angleSetter, requirements);
    }

    public Command storeEncoderOffsets(Subsystem ... requirements) {
        return new StoreEncoderOffsetsCommand(modules, requirements);
    }

    public Command startOdometryCalibration(Subsystem... requirements) {
        return new StartOdometryCalibrationCommand(modules, requirements);
    }

    public Command endOdometryCalibration(Subsystem... requirements) {
        return new EndOdometryCalibrationCommand(modules, requirements);
    }

    public Command calibrateCouplingRatio(Subsystem... requirements) {
        return new CalibrateCouplingRatioCommand(modules, Rotations.of(2), Volts.of(0.25), requirements);
    }
}
