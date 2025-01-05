package botsnax.swerve.phoenix;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import botsnax.system.Gearbox;
import botsnax.system.motor.phoenix.TalonSRXMotor;

public class TalonSRXSteering {
    public static Gearbox create(SwerveModuleConstants constants) {
        TalonSRX motor = new TalonSRX(constants.SteerMotorId);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        SlotConfiguration slotConfig = new SlotConfiguration();

        slotConfig.kP = constants.SteerMotorGains.kP;
        slotConfig.kI = constants.SteerMotorGains.kI;
        slotConfig.kD = constants.SteerMotorGains.kD;
        slotConfig.closedLoopPeakOutput = 0.75;

        config.peakCurrentLimit = 12; // the peak current, in amps
        config.peakCurrentDuration = 100; // the time at the peak current before the limit triggers, in ms
        config.continuousCurrentLimit = 7; // the current to maintain if the peak limit is triggered
        config.closedloopRamp = 0.1;
        config.slot0 = slotConfig;

        ErrorCode error = motor.configAllSettings(config);
        if (error != ErrorCode.OK) {
            System.err.println("Failed to configure motor " + constants.SteerMotorId + ": " + error);
        }

        int absolutePosition = motor.getSensorCollection().getPulseWidthPosition();
        error = motor.setSelectedSensorPosition(absolutePosition + constants.CANcoderOffset, 0, 1000);
        if (error != ErrorCode.OK) {
            System.err.println("Failed to set sensor position for motor " + constants.SteerMotorId + ": " + error);
        }

        return new TalonSRXMotor(motor).asGearbox();
    }
}
