package botsnax.swerve.phoenix;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.InvertedValue;
import botsnax.flywheel.Flywheel;
import botsnax.system.motor.phoenix.TalonFXMotor;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.CAN;

public class TalonFXDrive extends TalonFXMotor {
    private final SwerveModuleConstants<?, ?, ?> constants;

    public TalonFXDrive(TalonFX motor, boolean inverted, SwerveModuleConstants<?, ?, ?> constants) {
        super(motor, inverted, DCMotor.getKrakenX60(1));
        this.constants = constants;
    }

    public static Flywheel create(SwerveModuleConstants<?, ?, ?> constants, CANBus bus) {
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

        return new TalonFXDrive(motor, constants.DriveMotorInverted, constants);
    }
}
