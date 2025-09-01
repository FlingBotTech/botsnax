package botsnax.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.max;

public record PoseStdDev(Distance xStdDev, Distance yStdDev, Angle rStdDev) {
    public Distance getRadiusStdDev() {
        return Meters.of(max(xStdDev.in(Meters), yStdDev.in(Meters)));
    }

    public Matrix<N3, N1> asMatrix() {
        return new Matrix<>(N3.instance, N1.instance, new double[]{
                xStdDev.in(Meters),
                yStdDev.in(Meters),
                rStdDev.in(Radians)
        });
    }
}
