package botsnax.swerve;

import botsnax.swerve.listener.ModuleRequestListener;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import botsnax.flywheel.Flywheel;
import botsnax.system.Gearbox;
import botsnax.system.motor.AngleSetter;
import botsnax.system.motor.LinearVelocitySetter;
import botsnax.util.LinearAngularConversion;

import java.util.function.Function;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

public class SwerveModule {
    public record ApplyMode(
            Function<SwerveModule, SwerveModuleState> stateGetter,
            LinearVelocitySetter linearVelocitySetter,
            AngleSetter angleSetter,
            SwerveModuleStateOptimizer optimizer) {
    }

    private final Gearbox steering;
    private final Flywheel drive;
    private final LinearAngularConversion conversion;
    private final ModuleRequestListener moduleRequestListener;

    public SwerveModule(Gearbox steering, Flywheel drive, LinearAngularConversion conversion, ModuleRequestListener moduleRequestListener) {
        this.steering = steering;
        this.drive = drive;
        this.conversion = conversion;
        this.moduleRequestListener = moduleRequestListener;

        drive.setBrakeOnIdle();
    }

    public Flywheel getDrive() {
        return drive;
    }

    public Gearbox getSteering() {
        return steering;
    }

    public LinearAngularConversion getConversion() {
        return conversion;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                conversion.getLinearVelocity(drive.getVelocity()).in(MetersPerSecond),
                fromRadians(steering.getMotor().getAngle().in(Radians))
        );
    }

    public void apply(SwerveModuleState state, ApplyMode mode) {
        SwerveModuleState currentState = mode.stateGetter.apply(this);
        SwerveModuleState optimized = mode.optimizer().apply(state, currentState.angle);
        moduleRequestListener.onModuleRequest(optimized, currentState);
        mode.linearVelocitySetter().setVelocity(MetersPerSecond.of(optimized.speedMetersPerSecond), drive);
        mode.angleSetter().apply(optimized.angle, steering.getMotor());
    }
}
