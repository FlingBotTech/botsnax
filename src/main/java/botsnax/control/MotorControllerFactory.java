package botsnax.control;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class MotorControllerFactory {
    public static MotorController createConstantController(Voltage voltage) {
        return state -> voltage;
    }

    public static MotorController createSCurveController(
            Angle startAngle,
            Angle endAngle,
            AngularVelocity curveMaxVelocity,
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
            Angle startAngle,
            Angle endAngle,
            AngularVelocity curveMaxVelocity,
            Time dampingHalfLife,
            AngularVelocity velocityPerVolt,
            Time timeIncrement,
            SetpointController setpointController) {

        MotorController pathController = PathMotorController.ofPath(
                Profile.sCurve(
                        curveMaxVelocity.baseUnitMagnitude(),
                        startAngle.baseUnitMagnitude(),
                        endAngle.baseUnitMagnitude()),
                setpointController
        );

        MotorController damper = Damper.create(dampingHalfLife, velocityPerVolt, timeIncrement);

        return new CompositeMotorController(pathController , state -> Volts.of(0));
    }
}
