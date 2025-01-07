package botsnax.swerve.phoenix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.InvertedValue;
import botsnax.flywheel.Flywheel;
import botsnax.system.motor.phoenix.TalonFXMotor;

public class TalonFXDrive {
    public static Flywheel create(SwerveModuleConstants<?, ?, ?> constants, String bus) {
        TalonFX motor = new TalonFX(constants.DriveMotorId, bus);
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        talonConfigs.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        StatusCode response = motor.getConfigurator().apply(talonConfigs);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
        }

        return new TalonFXMotor(motor, constants.DriveMotorInverted);
    }
}
