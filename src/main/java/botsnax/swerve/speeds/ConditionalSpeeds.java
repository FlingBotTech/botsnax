package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

import javax.xml.stream.Location;
import java.util.List;
import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;

public class ConditionalSpeeds {
    public static Function<Pose2d, ChassisSpeeds> ifTrue(
            Function<Pose2d, Boolean> condition,
            Function<Pose2d, ChassisSpeeds> speeds) {
        return pose -> condition.apply(pose) ? speeds.apply(pose) : new ChassisSpeeds(0, 0, 0);
    }

    public static Function<Pose2d, ChassisSpeeds> ifTrue(
            Function<Pose2d, Boolean> condition,
            Function<Pose2d, ChassisSpeeds> trueSpeeds,
            Function<Pose2d, ChassisSpeeds> falseSpeeds) {
        return pose -> condition.apply(pose) ? trueSpeeds.apply(pose) : falseSpeeds.apply(pose);
    }

    public static Function<Pose2d, ChassisSpeeds> ifNotNear(List<Pose2d> poses, Distance distance, Function<Pose2d, ChassisSpeeds> speeds) {
        return pose -> {
            boolean isNear = false;

            for (Pose2d targetPose : poses) {
                if (targetPose.getTranslation().getDistance(pose.getTranslation()) < distance.in(Meters)) {
                    isNear = true;
                    break;
                }
            }

            return isNear ? new ChassisSpeeds(0, 0, 0) : speeds.apply(pose);
        };
    }

    public static Function<Pose2d, Boolean> ifInWedge(Translation2d center, Translation2d endpoint, Rotation2d angleFromAxis) {
        Translation2d axis = endpoint.minus(center);
        Rotation2d axisRotation = new Rotation2d(axis.getX(), axis.getY()).unaryMinus();
        double angleTan = angleFromAxis.getTan();

        return pose -> {
            Translation2d rotated = pose.getTranslation().minus(center).rotateBy(axisRotation);

            if (rotated.getX() > 0) {
                double tan = Math.abs(rotated.getY()) / rotated.getX();
                return tan < angleTan;
            } else {
                return false;
            }
        };
    }
}
