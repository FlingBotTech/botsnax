package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.pow;

public class ProportionalDriveSupplier implements ChassisSpeedsSupplier {
    private final Supplier<Double> vxSupplier;
    private final Supplier<Double> vySupplier;
    private final Supplier<Double> vrSupplier;
    private final double controllerDeadband;
    private final LinearVelocity maxSpeed;
    private final AngularVelocity maxAngularSpeed;

    public ProportionalDriveSupplier(Supplier<Double> vxSupplier, Supplier<Double> vySupplier, Supplier<Double> vrSupplier, double controllerDeadband, LinearVelocity maxSpeed, AngularVelocity maxAngularSpeed) {
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.vrSupplier = vrSupplier;
        this.controllerDeadband = controllerDeadband;
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
    }

    @Override
    public ChassisSpeeds get(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        double vx = vxSupplier.get();
        double vy = vySupplier.get();
        double vr = vrSupplier.get();
        boolean isInDeadband = pow(vx, 2) + pow(vy, 2) < pow(controllerDeadband, 2);
        double toApplyX = maxSpeed.times(isInDeadband ? 0 : vx).in(MetersPerSecond);
        double toApplyY = maxSpeed.times(isInDeadband ? 0 : vy).in(MetersPerSecond);
        double toApplyOmega = Math.abs(vr) > controllerDeadband ? maxAngularSpeed.times(vr).in(RadiansPerSecond) : 0;

        ChassisSpeeds speeds = ChassisSpeeds.discretize(new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega), parameters.updatePeriod);
//        ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()), parameters.updatePeriod);

        return speeds;
    }
}
