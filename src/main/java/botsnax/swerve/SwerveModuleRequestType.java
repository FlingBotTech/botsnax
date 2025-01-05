package botsnax.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import botsnax.flywheel.Flywheel;
import botsnax.system.motor.AngleSetter;
import botsnax.system.motor.MotorSystem;
import botsnax.system.motor.VelocitySetter;

public interface SwerveModuleRequestType {
    void apply(SwerveModuleState state, Flywheel drive, MotorSystem steer, VelocitySetter velocitySetter, AngleSetter angleSetter);
}
