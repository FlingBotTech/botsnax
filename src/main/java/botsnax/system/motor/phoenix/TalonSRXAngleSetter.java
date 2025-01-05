package botsnax.system.motor.phoenix;

import edu.wpi.first.math.geometry.Rotation2d;
import botsnax.system.motor.MotorSystem;

import static com.ctre.phoenix.motorcontrol.TalonSRXControlMode.Position;

public class TalonSRXAngleSetter {
    private static final double ENCODER_RESOLUTION = 4096;

    public static void applyPosition(Rotation2d angle, MotorSystem motor) {
        ((TalonSRXMotor) motor).get().set(Position, angle.getRotations() * ENCODER_RESOLUTION);
    }
}
