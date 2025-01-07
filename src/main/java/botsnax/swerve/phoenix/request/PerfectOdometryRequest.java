package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import botsnax.swerve.sim.SwerveSim;

public class PerfectOdometryRequest implements SwerveRequest {
    private final SwerveSim swerveSim;
    private final SwerveRequest request;

    public PerfectOdometryRequest(SwerveSim swerveSim, SwerveRequest request) {
        this.swerveSim = swerveSim;
        this.request = request;
    }

    @Override
    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        parameters.currentPose = swerveSim.getPose();
        return request.apply(parameters, modulesToApply);
    }
}
