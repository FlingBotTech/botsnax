package botsnax.swerve.phoenix;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PhoenixSwerveDrivetrain extends SwerveDrivetrain {
    private final SwerveDrivetrainConstants drivetrainConstants;
    private final SwerveModuleConstants[] modules;

    public PhoenixSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        this.drivetrainConstants = driveTrainConstants;
        this.modules = modules;
    }

    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return drivetrainConstants;
    }

    public SwerveModuleConstants[] getModuleConstants() {
        return modules;
    }

    public SwerveModule[] getModules() {
        return Modules;
    }

    public Translation2d[] getModuleLocations() {
        return m_moduleLocations;
    }

    public Pigeon2 getPigeon() {
        return m_pigeon2;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
}
