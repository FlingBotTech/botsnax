package botsnax.swerve.phoenix.request;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.pow;

public class ProportionalDriveSupplier implements ChassisSpeedsSupplier {
    private final Supplier<Double> vxSupplier;
    private final Supplier<Double> vySupplier;
    private final Supplier<Double> vrSupplier;
    private final double controllerDeadband;
    private final Measure<Velocity<Distance>> maxSpeed;
    private final Measure<Velocity<Angle>> maxAngularSpeed;

    public ProportionalDriveSupplier(Supplier<Double> vxSupplier, Supplier<Double> vySupplier, Supplier<Double> vrSupplier, double controllerDeadband, Measure<Velocity<Distance>> maxSpeed, Measure<Velocity<Angle>> maxAngularSpeed) {
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.vrSupplier = vrSupplier;
        this.controllerDeadband = controllerDeadband;
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
    }

    @Override
    public ChassisSpeeds get(SwerveRequest.SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
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
