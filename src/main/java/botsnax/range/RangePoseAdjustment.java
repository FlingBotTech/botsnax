//package botsnax.range;
//
//import botsnax.vision.PoseEstimate;
//import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.numbers.N1;
//import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.Distance;
//import edu.wpi.first.units.measure.Time;
//
//import java.util.Optional;
//
//import static edu.wpi.first.units.Units.Meters;
//import static edu.wpi.first.units.Units.Radians;
//
//public class RangePoseAdjustment {
//    public static Optional<PoseEstimate> snapTwoPoints(Pose2d robotPose, MountedRangeSensor leftSensor, MountedRangeSensor rightSensor, Pose2d surface) {
//        Optional<PositionEstimate> leftEstimateIfAny = leftSensor.getFieldRelativeMeasurementIfAny(robotPose);
//        Optional<PositionEstimate> rightEstimateIfAny = rightSensor.getFieldRelativeMeasurementIfAny(robotPose);
//
//        return leftEstimateIfAny.flatMap(leftEstimate ->
//                rightEstimateIfAny.map(rightEstimate -> {
//                    Pose2d snapped = snapTwoPoints(robotPose, leftEstimate.position(), rightEstimate.position(), surface);
//                    Distance stdDev = leftEstimate.stdDev().plus(rightEstimate.stdDev());
//                    Angle angularStdDev = Radians.of(stdDev.in(Meters) / 0.5); // Roughly half-meter robot radius
//                    Time timestamp = leftEstimate.timestamp().plus(rightEstimate.timestamp()).div(2);
//
//                    return new PoseEstimate(
//                            snapped,
//                            new Matrix<>(N3.instance, N1.instance, new double[]{
//                                    stdDev.in(Meters),
//                                    stdDev.in(Meters),
//                                    angularStdDev.in(Radians)
//                            }),
//                            timestamp
//                    );
//        }));
//    }
//
//    public static Pose2d snapOnePoint(Pose2d robotPose, Translation2d rangedPoint, Pose2d surface) {
//        Translation2d surfaceNormal = new Translation2d(1, 0).rotateBy(surface.getRotation());
//        double sensedPointDistance = dotProduct(rangedPoint.minus(surface.getTranslation()), surfaceNormal);
//        Translation2d displacement = surfaceNormal.times(-sensedPointDistance);
//
//        return new Pose2d(robotPose.getTranslation().plus(displacement), robotPose.getRotation());
//
////        return robotPose.transformBy(new Transform2d(displacement, Rotation2d.kZero));
//    }
//
//    private static Pose2d snapTwoPoints(Pose2d robotPose, Translation2d leftRangedPoint, Translation2d rightRangedPoint, Pose2d surface) {
//        Translation2d midRangedPoint = leftRangedPoint.plus(rightRangedPoint).div(2);
//        Pose2d translatedPose = snapOnePoint(robotPose, midRangedPoint, surface);
//        Translation2d delta = rightRangedPoint.minus(leftRangedPoint);
//        Rotation2d rotation = new Rotation2d(delta.getX(), delta.getY())
//                .rotateBy(Rotation2d.kCW_90deg)
////                .rotateBy(Rotation2d.kCCW_90deg)
//                .unaryMinus()
//                .rotateBy(surface.getRotation().rotateBy(Rotation2d.k180deg))
//                ;
//
//        return new Pose2d(translatedPose.getX(), translatedPose.getY(), rotation); // rotateAround(translatedPose, midRangedPoint, rotation);
//    }
//
//    private static Pose2d rotateAround(Pose2d pose, Translation2d center, Rotation2d rotation) {
//        return new Pose2d(
//                pose.getTranslation().rotateAround(center, rotation),
//                pose.getRotation() /*.plus(rotation) */
//        );
//    }
//
//    private static double dotProduct(Translation2d t1, Translation2d t2) {
//        return t1.getX() * t2.getX() + t1.getY() * t2.getY();
//    }
//}
