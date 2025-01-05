package botsnax.control;

import java.util.function.DoubleFunction;

import static java.lang.Math.*;

public class Profile {
    private static DoubleFunction<Double> sCurve(double maxVelocity, double endY) {
        final double endX = (1.5 * endY) / maxVelocity;
        final double c1 = 2.0 / 3;
        final double c2 = (-2 * maxVelocity) / pow(endX, 2);

        return x -> (x <= endX) ? c2 * pow(x, 2) * (c1 * x - endX) : endY;
    }

    public static DoubleFunction<Double> sCurve(double maxVelocity, double startY, double endY) {
        double deltaY = endY - startY;
        double signCorrectedMaxVelocity = signum(deltaY) * abs(maxVelocity);
        final DoubleFunction<Double> sCurve = sCurve(signCorrectedMaxVelocity, deltaY);

        return x -> sCurve.apply(x) + startY;
    }
}
