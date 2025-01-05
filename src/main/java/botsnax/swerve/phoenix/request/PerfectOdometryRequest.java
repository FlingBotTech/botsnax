package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import botsnax.swerve.sim.SwerveSim;

public class PerfectOdometryRequest implements SwerveRequest {
    private final SwerveSim swerveSim;
    private final SwerveRequest request;

    public PerfectOdometryRequest(SwerveSim swerveSim, SwerveRequest request) {
        this.swerveSim = swerveSim;
        this.request = request;
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        parameters.currentPose = swerveSim.getPose();
        return request.apply(parameters, modulesToApply);
    }
}
