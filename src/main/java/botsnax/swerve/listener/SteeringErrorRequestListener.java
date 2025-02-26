package botsnax.swerve.listener;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SteeringErrorRequestListener {
    public static ModuleRequestListener of(int moduleId) {
        DoublePublisher errorPublisher = NetworkTableInstance.getDefault().getDoubleTopic("SteeringError" + moduleId).publish();

        return (requested, current) ->
                errorPublisher.set(Math.abs(requested.angle.minus(current.angle).getDegrees()));
    }
}
