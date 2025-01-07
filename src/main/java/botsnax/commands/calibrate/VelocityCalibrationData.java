package botsnax.commands.calibrate;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Volts;

public class VelocityCalibrationData {
    private record VelocityVoltage(AngularVelocity velocity, Voltage voltage) { }

    private final List<VelocityVoltage> results = new ArrayList<>();

    public void add(AngularVelocity velocity, Voltage voltage) {
        results.add(new VelocityVoltage(velocity, voltage));
    }

    public VelocityCalibration solve() {
        return solve(results);
    }

    public VelocityCalibration solvePositive() {
        List<VelocityVoltage> positives = results.stream().filter(vv -> vv.voltage.baseUnitMagnitude() > 0).toList();

        return solve(positives);
    }

    public VelocityCalibration solveNegative() {
        List<VelocityVoltage> negatives = results.stream().filter(vv -> vv.voltage.baseUnitMagnitude() < 0).toList();

        return solve(negatives);
    }

    private VelocityCalibration solve(List<VelocityVoltage> results) {
        SimpleMatrix A = new SimpleMatrix(results.size(), 2);
        SimpleMatrix B = new SimpleMatrix(results.size(), 1);

        for (int i = 0; i < results.size(); i++) {
            A.set(i, 0, 1);
            A.set(i, 1, results.get(i).velocity().baseUnitMagnitude());
            B.set(i, 0, results.get(i).voltage().baseUnitMagnitude());
        }

        SimpleMatrix solution = A.pseudoInverse().mult(B);

        return new VelocityCalibration(
                solution.get(1, 0),
                Volts.of(solution.get(0, 0)));
    }
}
