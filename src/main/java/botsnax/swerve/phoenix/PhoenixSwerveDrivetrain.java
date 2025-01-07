package botsnax.swerve.phoenix;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PhoenixSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    private final SwerveDrivetrainConstants drivetrainConstants;
    private final SwerveModuleConstants<?, ?, ?>[] modules;

    public PhoenixSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(TalonFX :: new, TalonFX :: new, CANcoder :: new, drivetrainConstants, modules);
        this.drivetrainConstants = drivetrainConstants;
        this.modules = modules;
    }

    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return drivetrainConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return modules;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }
}
