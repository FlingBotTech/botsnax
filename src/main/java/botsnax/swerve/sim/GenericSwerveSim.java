package botsnax.swerve.sim;

import botsnax.swerve.SwerveModule;
import botsnax.system.Gyro;
import botsnax.system.GyroSim;
import botsnax.system.encoder.EncoderSim;
import botsnax.system.motor.MotorSim;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class GenericSwerveSim {
    protected static class SwerveModuleSim {
        /** Reference to motor simulation for drive motor */
        public final DCMotorSim DriveMotor;
        /** Reference to motor simulation for the steer motor */
        public final DCMotorSim SteerMotor;
        /** Reference to steer gearing for updating encoder */
        public final double DriveGearing;
        /** Reference to steer gearing for updating encoder */
        public final double SteerGearing;
        /** Voltage necessary for the drive motor to overcome friction */
        public final double DriveFrictionVoltage;
        /** Voltage necessary for the steer motor to overcome friction */
        public final double SteerFrictionVoltage;
        /** Whether the drive motor is inverted */
        public final boolean DriveMotorInverted;
        /** Whether the steer motor is inverted */
        public final boolean SteerMotorInverted;
        /** Whether the azimuth encoder is inverted */
        public final boolean EncoderInverted;

        public SwerveModuleSim(
                DCMotor driveMotor,
                DCMotor steerMotor,
                double driveGearing,
                double driveInertia,
                double driveFrictionVoltage,
                boolean driveMotorInverted,
                double steerGearing,
                double steerInertia,
                double steerFrictionVoltage,
                boolean steerMotorInverted,
                boolean encoderInverted
        ) {
            DriveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveMotor, driveInertia, driveGearing), driveMotor);
            SteerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(steerMotor, steerInertia, steerGearing), steerMotor);
            DriveGearing = driveGearing;
            SteerGearing = steerGearing;
            DriveFrictionVoltage = driveFrictionVoltage;
            SteerFrictionVoltage = steerFrictionVoltage;
            DriveMotorInverted = driveMotorInverted;
            SteerMotorInverted = steerMotorInverted;
            EncoderInverted = encoderInverted;
        }
    }

    protected final GyroSim gyroSim;
    protected final SwerveModuleSim[] moduleSims;

    private final IdealizedSwerveSim swerveSim;
    protected final SwerveDriveKinematics kinematics;

    public GenericSwerveSim(
            IdealizedSwerveSim swerveSim,
            Gyro gyro,
            SwerveModule[] modules,
            SwerveDriveKinematics kinematics,
            SwerveModuleConstants<?, ?, ?>... moduleConstants
    ) {
        gyroSim = gyro.getSim();
        moduleSims = new SwerveModuleSim[moduleConstants.length];
        for (int i = 0; i < moduleSims.length; ++i) {
            moduleSims[i] = new SwerveModuleSim(
                    modules[i].getDrive().getDCMotor(),
                    modules[i].getSteering().getMotor().getDCMotor(),
                    moduleConstants[i].DriveMotorGearRatio, moduleConstants[i].DriveInertia,
                    moduleConstants[i].DriveFrictionVoltage, moduleConstants[i].DriveMotorInverted,
                    moduleConstants[i].SteerMotorGearRatio, moduleConstants[i].SteerInertia,
                    moduleConstants[i].SteerFrictionVoltage, moduleConstants[i].SteerMotorInverted,
                    moduleConstants[i].EncoderInverted
            );
        }

        this.swerveSim = swerveSim;
        this.kinematics = kinematics;
    }

    public final void update(Time dt, Voltage supplyVoltage, SwerveModule... modulesToApply) {
        SwerveModuleState[] states = new SwerveModuleState[moduleSims.length];
        /* Update our sim devices */
        for (int i = 0; i < moduleSims.length; ++i) {
            MotorSim driveMotor = (MotorSim) modulesToApply[i].getDrive().getSim();
            MotorSim steerMotor = modulesToApply[i].getSteering().getMotor().getSim();
            EncoderSim encoder = modulesToApply[i].getSteering().getOutputEncoder().getSim();

            driveMotor.setInverted(moduleSims[i].DriveMotorInverted);
            steerMotor.setInverted(moduleSims[i].SteerMotorInverted);
            encoder.setInverted(moduleSims[i].EncoderInverted);

            driveMotor.setSupplyVoltage(supplyVoltage);
            steerMotor.setSupplyVoltage(supplyVoltage);
            encoder.setSupplyVoltage(supplyVoltage);

            moduleSims[i].DriveMotor.setInputVoltage(addFriction(driveMotor.getMotorVoltage().in(Volts), moduleSims[i].DriveFrictionVoltage));
            moduleSims[i].SteerMotor.setInputVoltage(addFriction(steerMotor.getMotorVoltage().in(Volts), moduleSims[i].SteerFrictionVoltage));

            moduleSims[i].DriveMotor.update(dt.in(Seconds));
            moduleSims[i].SteerMotor.update(dt.in(Seconds));

            driveMotor.setRawPosition(Rotations.of(moduleSims[i].DriveMotor.getAngularPositionRotations()).times(moduleSims[i].DriveGearing));
            driveMotor.setVelocity(RPM.of(moduleSims[i].DriveMotor.getAngularVelocityRPM()).times(moduleSims[i].DriveGearing));

            steerMotor.setRawPosition(Rotations.of(moduleSims[i].SteerMotor.getAngularPositionRotations()).times(moduleSims[i].SteerGearing));
            steerMotor.setVelocity(RPM.of(moduleSims[i].SteerMotor.getAngularVelocityRPM()).times(moduleSims[i].SteerGearing));

            /* azimuth encoders see the mechanism, so don't account for the steer gearing */
            encoder.setRawPosition(Rotations.of(moduleSims[i].SteerMotor.getAngularPositionRotations()));
            encoder.setVelocity(RPM.of(moduleSims[i].SteerMotor.getAngularVelocityRPM()));

            states[i] = modulesToApply[i].getState();
        }

        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(states);
        swerveSim.update(chassisSpeeds, dt);
        gyroSim.setRawYaw(Radians.of(swerveSim.getPose().getRotation().getRadians()));
        gyroSim.setAngularVelocityZ(RadiansPerSecond.of(swerveSim.getVelocity().omegaRadiansPerSecond));
    }

    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage Voltage output by the motor
     * @param frictionVoltage Voltage required to overcome friction
     * @return Friction-dampened motor voltage
     */
    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }
}
