package botsnax.swerve;

import botsnax.swerve.speeds.ThresholdParams;
import botsnax.util.LinearAngularConversion;
import botsnax.vision.PoseStdDev;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class SwerveDrivetrainValues {
    private final SwerveModuleConstants<?, ?, ?> module;
    private final SwerveModule.DriveRequestType driveRequestType;
    private final Time systemLatency;
    private final LinearVelocity minDriveSpeed;
    private final Time linearStopTime;
    private final Time angularStopTime;

    public SwerveDrivetrainValues(
            SwerveModuleConstants<?, ?, ?> module,
            SwerveModule.DriveRequestType driveRequestType, Time systemLatency, LinearVelocity minDriveSpeed, Time linearStopTime, Time angularStopTime) {
        this.module = module;
        this.driveRequestType = driveRequestType;
        this.systemLatency = systemLatency;
        this.minDriveSpeed = minDriveSpeed;
        this.linearStopTime = linearStopTime;
        this.angularStopTime = angularStopTime;
    }

    public Distance getSteerAxisToCenterDistance() {
        return getSteerAxisToCenterDistance(module);
    }

    public LinearVelocity getMaxSpeedAt12Volts() {
        return getMaxSpeedAtVoltage(driveRequestType, module, Volts.of(12));
    }

    public AngularVelocity getMaxRotationSpeedAt12Volts() {
        return RadiansPerSecond.of(getMaxSpeedAt12Volts().in(MetersPerSecond) / getSteerAxisToCenterDistance().in(Meters));
    }

    public LinearVelocity getMinDriveSpeed() {
        return minDriveSpeed;
    }

    public AngularVelocity getMinRotationSpeed() {
        return RadiansPerSecond.of(getMinDriveSpeed().baseUnitMagnitude() / getSteerAxisToCenterDistance().baseUnitMagnitude());
    }

    public ThresholdParams getThresholdParams(PoseStdDev poseStdDev, Time sensorLatency) {
        return new ThresholdParams(
                minDriveSpeed,
                systemLatency,
                poseStdDev,
                sensorLatency,
                getSteerAxisToCenterDistance()
        );
    }

    public static Distance getSteerAxisToCenterDistance(SwerveModuleConstants<?, ?, ?> moduleConstants) {
        return Meters.of(Math.hypot(moduleConstants.LocationX, moduleConstants.LocationY));
    }

    public static LinearVelocity getMaxSpeedAtVoltage(SwerveModule.DriveRequestType driveRequestType, SwerveModuleConstants<?, ?, ?> moduleConstants, Voltage voltage) {
        if (driveRequestType.equals(SwerveModule.DriveRequestType.Velocity)) {
            AngularVelocity wheelRotationSpeed = RotationsPerSecond.of(voltage.in(Volts) / moduleConstants.DriveMotorGains.kV);
            LinearAngularConversion conversion = LinearAngularConversion.ofWheelRadiusGearRatio(Meters.of(moduleConstants.WheelRadius), moduleConstants.DriveMotorGearRatio);

            return conversion.getLinearVelocity(wheelRotationSpeed);
        } else {
            return MetersPerSecond.of(moduleConstants.SpeedAt12Volts).times(voltage.in(Volts) / 12.0);
        }
    }

    public Time getAngularStopTime() {
        return angularStopTime;
    }

    public Time getLinearStopTime() {
        return linearStopTime;
    }
}
