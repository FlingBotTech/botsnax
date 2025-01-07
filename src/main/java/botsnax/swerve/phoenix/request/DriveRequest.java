package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveRequest implements SwerveRequest
{
    protected SwerveModule.DriveRequestType driveRequestType;
    protected SwerveModule.SteerRequestType steerRequestType;
    protected ChassisSpeedsSupplier chassisSpeedsSupplier;
    private final ModuleRequest moduleRequest;

    public DriveRequest(SwerveModule.DriveRequestType driveRequestType, SwerveModule.SteerRequestType steerRequestType, ChassisSpeedsSupplier chassisSpeedsSupplier) {
        this.driveRequestType = driveRequestType;
        this.steerRequestType = steerRequestType;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        moduleRequest = new ModuleRequest()
                .withSteerRequest(steerRequestType)
                .withDriveRequest(driveRequestType);
    }

    @Override
    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        ChassisSpeeds speeds = chassisSpeedsSupplier.get(parameters, modulesToApply);
        SwerveModuleState[] states = parameters.kinematics.toSwerveModuleStates(speeds);

        for(int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(moduleRequest.withState(states[i]));
        }

        return StatusCode.OK;
    }
}
