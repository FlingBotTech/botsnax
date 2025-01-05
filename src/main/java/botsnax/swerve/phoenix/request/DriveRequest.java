package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveRequest implements SwerveRequest
{
    protected SwerveModule.DriveRequestType driveRequestType;
    protected SwerveModule.SteerRequestType steerRequestType;
    protected ChassisSpeedsSupplier chassisSpeedsSupplier;

    public DriveRequest(SwerveModule.DriveRequestType driveRequestType, SwerveModule.SteerRequestType steerRequestType, ChassisSpeedsSupplier chassisSpeedsSupplier) {
        this.driveRequestType = driveRequestType;
        this.steerRequestType = steerRequestType;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        ChassisSpeeds speeds = chassisSpeedsSupplier.get(parameters, modulesToApply);
        SwerveModuleState[] states = parameters.kinematics.toSwerveModuleStates(speeds);

        for(int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], this.driveRequestType, this.steerRequestType);
        }

        return StatusCode.OK;
    }
}
