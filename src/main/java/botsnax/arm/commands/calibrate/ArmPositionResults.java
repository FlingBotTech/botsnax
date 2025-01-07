package botsnax.arm.commands.calibrate;

import botsnax.arm.commands.AwaitStabilityCommand;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Voltage;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.*;
import static java.util.Arrays.stream;
import static org.ejml.simple.SimpleMatrix.filled;

public class ArmPositionResults {
    private static final AngleUnit ANGLE_UNITS = Degrees;

    private final List<AwaitStabilityCommand.Result> results;

    private ArmPositionResults(List<AwaitStabilityCommand.Result> results) {
        this.results = results;
    }

    public ArmPositionResults() {
        this(new ArrayList<>());
    }

    public void add(AwaitStabilityCommand.Result result) {
        results.add(result);
    }

    private ArmGravityController iterate(ArmGravityController armGravityController) {
        Voltage gain = armGravityController.gain();
        double offset = armGravityController.offset().in(Radians);

        SimpleMatrix error = new SimpleMatrix(1, this.results.size());
        SimpleMatrix partials = new SimpleMatrix(this.results.size(), 2);

        for (int i = 0; i < results.size(); i++) {
            AwaitStabilityCommand.Result r = results.get(i);
            double position = r.angle().in(Radians);

            error.set(0, i, (r.voltage().minus(gain.times(cos(position - offset)))).baseUnitMagnitude());
            partials.set(i, 0, cos(position - offset));
            partials.set(i, 1, gain.times(sin(position - offset)).baseUnitMagnitude());
        }

        SimpleMatrix errorSquared = error.mult(error.transpose());
        SimpleMatrix jacobian = error.mult(partials).elementMult(filled(1, 2, -2));
        SimpleMatrix solution = jacobian
                .pseudoInverse()
                .mult(errorSquared)
                .elementMult(filled(2, 1, -1));

        Voltage gain2 = gain.plus(Volts.of(solution.get(0, 0)));
        double offset2 = solution.get(1, 0) + offset;

        return new ArmGravityController(gain2, Radians.of(offset2));
    }

    private static Stats getStats(double[] values) {
        double mean = stream(values).average().orElseThrow();
        double variance = stream(values).map(x -> pow(x - mean, 2)).average().orElseThrow();

        return new Stats(mean, sqrt(variance));
    }

    private static Stats getFilteredStats(double[] values) {
        Stats stats = getStats(values);
        double[] filtered = stream(values)
                .filter(x -> abs(x - stats.getMean()) <= stats.getStandardDeviation())
                .toArray();

        return getStats(filtered);
    }

    private static ArmGravityController getFilteredCalibration(List<ArmGravityController> calibrations) {
        double[] offsets = calibrations.stream()
                .mapToDouble(c -> c.offset().in(ANGLE_UNITS))
                .toArray();
        Stats offsetStats = getFilteredStats(offsets);

        double[] gains = calibrations.stream()
                .map(ArmGravityController::gain)
                .mapToDouble(Voltage::baseUnitMagnitude)
                .toArray();
        Stats gainStats = getFilteredStats(gains);

        return new ArmGravityController(Volts.of(gainStats.getMean()), ANGLE_UNITS.of(offsetStats.getMean()));
    }

    public ArmGravityController solveForGravity() {
        double maxVoltage = results.stream().mapToDouble(result -> result.voltage().baseUnitMagnitude()).max().orElseThrow();
        AwaitStabilityCommand.Result maxResult = results.stream()
                .filter(result -> result.voltage().baseUnitMagnitude() == maxVoltage).findFirst().orElseThrow();
        List<AwaitStabilityCommand.Result> others = results.stream()
                .filter(result -> result.voltage().baseUnitMagnitude() != maxVoltage).toList();
        List<AwaitStabilityCommand.Result> resultsToUse = Arrays.asList(
                others.get(0),
                maxResult,
                others.get(others.size() - 1)
        );

        return new ArmPositionResults(resultsToUse).doSolveForGravity();
    }

    private ArmGravityController doSolveForGravity() {
        ArrayList<ArmGravityController> calibrations = new ArrayList<>();
        ArmGravityController previous = new ArmGravityController(Volts.of(0),  ANGLE_UNITS.of(0));

        for (int i = 0; i < 100; i++) {
            ArmGravityController calibration = iterate(previous);
            calibrations.add(calibration);
            previous = calibration;
        }

         return getFilteredCalibration(calibrations);
    }

    public double solveForGearRatio() {
        SimpleMatrix A = new SimpleMatrix(results.size(), 1);
        SimpleMatrix B = new SimpleMatrix(results.size(), 1);

        for (int i = 0; i < results.size(); i++) {
            A.set(i, 0, results.get(i).angle().baseUnitMagnitude());
            B.set(i, 0, results.get(i).motorAngle().baseUnitMagnitude());
        }

        SimpleMatrix solution = A.pseudoInverse().mult(B);

        return solution.get(0, 0);
    }

    private static class Stats {
        private final double mean;
        private final double standardDeviation;

        public Stats(double mean, double standardDeviation) {
            this.mean = mean;
            this.standardDeviation = standardDeviation;
        }

        public double getMean() {
            return mean;
        }

        public double getStandardDeviation() {
            return standardDeviation;
        }
    }
}
