package botsnax.control;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;

public class MotorControllerFactory {
    public static MotorController createConstantController(double voltage) {
        return state -> voltage;
    }

    public static MotorController createSCurveController(
            Measure<Angle> startAngle,
            Measure<Angle> endAngle,
            Measure<Velocity<Angle>> curveMaxVelocity,
            SetpointController setpointController) {

        MotorController pathController = PathMotorController.ofPath(
                Profile.sCurve(
                        curveMaxVelocity.baseUnitMagnitude(),
                        startAngle.baseUnitMagnitude(),
                        endAngle.baseUnitMagnitude()),
                setpointController
        );

        return pathController;
    }

    public static MotorController createSCurveController(
            Measure<Angle> startAngle,
            Measure<Angle> endAngle,
            Measure<Velocity<Angle>> curveMaxVelocity,
            Measure<Time> dampingHalfLife,
            Measure<Velocity<Angle>> velocityPerVolt,
            Measure<Time> timeIncrement,
            SetpointController setpointController) {

        MotorController pathController = PathMotorController.ofPath(
                Profile.sCurve(
                        curveMaxVelocity.baseUnitMagnitude(),
                        startAngle.baseUnitMagnitude(),
                        endAngle.baseUnitMagnitude()),
                setpointController
        );

        MotorController damper = Damper.create(dampingHalfLife, velocityPerVolt, timeIncrement);

        return new CompositeMotorController(pathController , state -> 0);
    }
}
