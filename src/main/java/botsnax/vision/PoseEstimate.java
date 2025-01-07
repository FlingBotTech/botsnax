package botsnax.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public record PoseEstimate(Pose2d pose, Matrix<N3, N1> stdDevs, Time timestamp) {
}
