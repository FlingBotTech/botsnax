package botsnax.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.XboxController;

import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.pow;

public class ControllerSpeeds {
    public static Function<Pose2d, ChassisSpeeds> of(
            Supplier<Double> vxSupplier,
            Supplier<Double> vySupplier,
            Supplier<Double> vrSupplier,
            double controllerDeadband,
            LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed
    ) {
        return pose -> {
            double vx = vxSupplier.get();
            double vy = vySupplier.get();
            double vr = vrSupplier.get();
            boolean isInDeadband = pow(vx, 2) + pow(vy, 2) < pow(controllerDeadband, 2);
            double toApplyX = maxSpeed.times(isInDeadband ? 0 : vx).in(MetersPerSecond);
            double toApplyY = maxSpeed.times(isInDeadband ? 0 : vy).in(MetersPerSecond);
            double toApplyOmega = Math.abs(vr) > controllerDeadband ? maxAngularSpeed.times(vr).in(RadiansPerSecond) : 0;

            return new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);
        };
    }

    public static Function<Pose2d, ChassisSpeeds> of(
            XboxController controller,
            double controllerDeadband,
            LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed) {
        return ControllerSpeeds.of(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                controllerDeadband,
                maxSpeed,
                maxAngularSpeed
        );
    }
}
