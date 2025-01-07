package botsnax.swerve.phoenix.request;

import botsnax.control.ReactionTimeVelocityController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
    public ChassisSpeeds get(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Translation2d delta = targetPosition.minus(parameters.currentPose.getTranslation());
        Angle angleToTarget = Radians.of(Math.atan2(delta.getY(), delta.getX()));
        Angle yawAngle = Radians.of(parameters.currentPose.getRotation().getRadians());
        Angle error = Radians.of(MathUtil.inputModulus(angleToTarget.minus(yawAngle).in(Radians), -Math.PI, Math.PI));
        AngularVelocity velocity = velocityCalculator.calculate(error);
        ChassisSpeeds speeds = chassisSpeedsSupplier.get(parameters, modulesToApply);

        speeds.omegaRadiansPerSecond = velocity.in(RadiansPerSecond);

        return speeds;
    }
}
