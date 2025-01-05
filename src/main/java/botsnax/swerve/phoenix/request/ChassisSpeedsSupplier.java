package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedsSupplier {
    ChassisSpeeds get(SwerveRequest.SwerveControlRequestParameters parameters, SwerveModule... modulesToApply);
}
