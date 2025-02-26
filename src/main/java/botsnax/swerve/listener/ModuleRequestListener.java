package botsnax.swerve.listener;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleRequestListener {
    void onModuleRequest(SwerveModuleState requestedState, SwerveModuleState currentState);
}
