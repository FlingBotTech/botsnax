package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedsSupplier {
    ChassisSpeeds get(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply);
}
