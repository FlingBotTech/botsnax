package botsnax.swerve.phoenix;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import botsnax.swerve.sim.IdealizedSwerveSim;

import static edu.wpi.first.units.Units.Seconds;

public class PhoenixSwerveSim extends SimSwerveDrivetrain {
    private final IdealizedSwerveSim idealizedSim;

    public PhoenixSwerveSim(IdealizedSwerveSim idealizedSim,
                            Translation2d[] wheelLocations,
                            Pigeon2 pigeon,
                            SwerveDrivetrainConstants driveConstants,
                            SwerveModuleConstants... moduleConstants) {
        super(wheelLocations, pigeon, driveConstants, moduleConstants);
        this.idealizedSim = idealizedSim;
    }

    public void updateWithIdealized(double dtSeconds, double supplyVoltage, SwerveModule... modulesToApply) {
        if (m_modules.length != ModuleCount) return;

        SwerveModuleState[] states = new SwerveModuleState[ModuleCount];
        /* Update our sim devices */
        for (int i = 0; i < ModuleCount; ++i) {
            TalonFXSimState steerMotor = modulesToApply[i].getSteerMotor().getSimState();
            TalonFXSimState driveMotor = modulesToApply[i].getDriveMotor().getSimState();
            CANcoderSimState cancoder = modulesToApply[i].getCANcoder().getSimState();

            steerMotor.Orientation = m_modules[i].SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            driveMotor.Orientation = m_modules[i].DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

            steerMotor.setSupplyVoltage(supplyVoltage);
            driveMotor.setSupplyVoltage(supplyVoltage);
            cancoder.setSupplyVoltage(supplyVoltage);

            m_modules[i].SteerMotor.setInputVoltage(addFriction(steerMotor.getMotorVoltage(), m_modules[i].SteerFrictionVoltage));
            m_modules[i].DriveMotor.setInputVoltage(addFriction(driveMotor.getMotorVoltage(), m_modules[i].DriveFrictionVoltage));

            m_modules[i].SteerMotor.update(dtSeconds);
            m_modules[i].DriveMotor.update(dtSeconds);

            steerMotor.setRawRotorPosition(m_modules[i].SteerMotor.getAngularPositionRotations() * m_modules[i].SteerGearing);
            steerMotor.setRotorVelocity(m_modules[i].SteerMotor.getAngularVelocityRPM() / 60.0 * m_modules[i].SteerGearing);

            /* CANcoders see the mechanism, so don't account for the steer gearing */
            cancoder.setRawPosition(m_modules[i].SteerMotor.getAngularPositionRotations());
            cancoder.setVelocity(m_modules[i].SteerMotor.getAngularVelocityRPM() / 60.0);

            driveMotor.setRawRotorPosition(m_modules[i].DriveMotor.getAngularPositionRotations() * m_modules[i].DriveGearing);
            driveMotor.setRotorVelocity(m_modules[i].DriveMotor.getAngularVelocityRPM() / 60.0 * m_modules[i].DriveGearing);

            states[i] = modulesToApply[i].getCurrentState();
        }

        ChassisSpeeds chassisSpeeds = Kinem.toChassisSpeeds(states);
        idealizedSim.update(chassisSpeeds, Seconds.of(dtSeconds));
        PigeonSim.setRawYaw(idealizedSim.getPose().getRotation().getDegrees());
    }
}
