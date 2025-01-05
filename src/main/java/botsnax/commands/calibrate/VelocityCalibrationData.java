package botsnax.commands.calibrate;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class VelocityCalibrationData {
    private record VelocityVoltage(Measure<Velocity<Angle>> velocity, double voltage) { }

    private final List<VelocityVoltage> results = new ArrayList<>();

    public void add(Measure<Velocity<Angle>> velocity, double voltage) {
        results.add(new VelocityVoltage(velocity, voltage));
    }

    public VelocityCalibration solve() {
        return solve(results);
    }

    public VelocityCalibration solvePositive() {
        List<VelocityVoltage> positives = results.stream().filter(vv -> vv.voltage > 0).toList();

        return solve(positives);
    }

    public VelocityCalibration solveNegative() {
        List<VelocityVoltage> negatives = results.stream().filter(vv -> vv.voltage < 0).toList();

        return solve(negatives);
    }

    private VelocityCalibration solve(List<VelocityVoltage> results) {
        SimpleMatrix A = new SimpleMatrix(results.size(), 2);
        SimpleMatrix B = new SimpleMatrix(results.size(), 1);

        for (int i = 0; i < results.size(); i++) {
            A.set(i, 0, 1);
            A.set(i, 1, results.get(i).velocity().baseUnitMagnitude());
            B.set(i, 0, results.get(i).voltage());
        }

        SimpleMatrix solution = A.pseudoInverse().mult(B);

        return new VelocityCalibration(
                solution.get(1, 0),
                solution.get(0, 0));
    }
}
