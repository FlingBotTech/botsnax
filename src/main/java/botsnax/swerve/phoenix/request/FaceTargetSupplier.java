package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import botsnax.control.ReactionTimeVelocityController;

import static edu.wpi.first.units.Units.*;

public class FaceTargetSupplier implements ChassisSpeedsSupplier {
    private final ChassisSpeedsSupplier chassisSpeedsSupplier;
    private final ReactionTimeVelocityController velocityCalculator;
    private final Translation2d targetPosition;

    public FaceTargetSupplier(ChassisSpeedsSupplier chassisSpeedsSupplier, ReactionTimeVelocityController velocityCalculator, Translation2d targetPosition) {
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.velocityCalculator = velocityCalculator;
        this.targetPosition = targetPosition;
    }

    @Override
    public ChassisSpeeds get(SwerveRequest.SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        Translation2d delta = targetPosition.minus(parameters.currentPose.getTranslation());
        Measure<Angle> angleToTarget = Radians.of(Math.atan2(delta.getY(), delta.getX()));
        Measure<Angle> yawAngle = Radians.of(parameters.currentPose.getRotation().getRadians());
        Measure<Angle> error = Radians.of(MathUtil.inputModulus(angleToTarget.minus(yawAngle).in(Radians), -Math.PI, Math.PI));
        Measure<Velocity<Angle>> velocity = velocityCalculator.calculate(error);
        ChassisSpeeds speeds = chassisSpeedsSupplier.get(parameters, modulesToApply);

        speeds.omegaRadiansPerSecond = velocity.in(RadiansPerSecond);

        return speeds;
    }
}
