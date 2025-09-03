package botsnax.swerve.factory;

import botsnax.swerve.SwerveModule;
import botsnax.swerve.SwerveModuleStateOptimizer;
import botsnax.swerve.SwerveOdometryUpdater;
import botsnax.swerve.listener.SteeringErrorRequestListener;
import botsnax.swerve.phoenix.TalonFXDrive;
import botsnax.swerve.phoenix.TalonSRXSteering;
import botsnax.swerve.sim.PerfectSteering;
import botsnax.system.Gyro;
import botsnax.system.motor.VelocitySetter;
import botsnax.system.motor.phoenix.PigeonGyro;
import botsnax.system.motor.phoenix.TalonFXMotor;
import botsnax.system.motor.phoenix.TalonSRXMotor;
import botsnax.util.LinearAngularConversion;
import botsnax.util.PeriodicUpdater;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Timer;

import static botsnax.system.motor.phoenix.TalonSRXMotor.PID_CONTROL;
import static com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_2_Feedback0;
import static com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static edu.wpi.first.wpilibj.Threads.setCurrentThreadPriority;

public class TalonSRXSwerveDrivetrainFactory extends GenericSwerveDrivetrainFactory {
    protected static final int START_THREAD_PRIORITY = 1;
    private static final int DEFAULT_UPDATE_FREQUENCY = 100;

    private final VelocitySetter velocitySetter;

    public TalonSRXSwerveDrivetrainFactory(Mass mass, VelocitySetter velocitySetter, SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?> ... moduleConstants) {
        super(mass, drivetrainConstants, moduleConstants);
        this.velocitySetter = velocitySetter;
    }

    public TalonSRXSwerveDrivetrainFactory(Mass mass, DCMotor driveMotorType, SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?> ... moduleConstants) {
        this(mass, VelocitySetter.createOpenLoop(driveMotorType), drivetrainConstants, moduleConstants);
    }

    @Override
    protected SwerveModule.ApplyMode getDefaultApplyMode() {
        return new SwerveModule.ApplyMode(
                SwerveModule :: getState,
                this.velocitySetter,
                PID_CONTROL,
                new SwerveModuleStateOptimizer()
        );
    }

    @Override
    protected SwerveModulePosition getPosition(SwerveModule module) {
        TalonFXMotor driveMotor = (TalonFXMotor) module.getDrive();

        Angle drivePosition = (Angle) BaseStatusSignal.getLatencyCompensatedValue(driveMotor.get().getPosition(), driveMotor.get().getVelocity());
        Distance distance = module.getConversion().getDistance(drivePosition);
        Rotation2d angle = fromRadians(module.getSteering().getMotor().getAngle().in(Radians));

        return new SwerveModulePosition(distance.in(Meters), angle);
    }

    @Override
    protected Gyro createGyro() {
        return new PigeonGyro(new Pigeon2(drivetrainConstants.Pigeon2Id));
    }

    @Override
    protected SwerveModule createModule(int moduleId, SwerveModuleConstants<?, ?, ?> constants) {
        return new SwerveModule(
                isSimulation() ?
                        PerfectSteering.createGearbox() :
                        TalonSRXSteering.create(constants, DCMotor.getVex775Pro(1)),
                TalonFXDrive.create(constants, ""),
                LinearAngularConversion.ofWheelRadiusGearRatio(
                        Inches.of(constants.WheelRadius),
                        constants.DriveMotorGearRatio),
                SteeringErrorRequestListener.of(moduleId)
        );
    }

    @Override
    protected PeriodicUpdater<SwerveOdometryUpdater> createPeriodicUpdater(SwerveOdometryUpdater odometryUpdater, Gyro gyro, SwerveModule[] modules) {
        return new PeriodicUpdater<>(DEFAULT_UPDATE_FREQUENCY, odometryUpdater) {
            @Override
            protected void init() {
                setCurrentThreadPriority(true, START_THREAD_PRIORITY);

                Pigeon2 pigeon = ((PigeonGyro) gyro).get();
                setUpdateFrequency(getUpdateFrequency(), pigeon.getYaw(), pigeon.getAngularVelocityZWorld());

                for (SwerveModule module : modules) {
                    TalonFX driveMotor = ((TalonFXMotor) module.getDrive()).get();
                    setUpdateFrequency(getUpdateFrequency(), driveMotor.getPosition(), driveMotor.getVelocity());

                    if (module.getSteering().getMotor() instanceof TalonSRXMotor talonSRXMotor) {
                        setUpdatePeriod(getUpdateFrequency(), talonSRXMotor.get(), Status_2_Feedback0, Status_3_Quadrature);
                    }
                }
            }

            @Override
            protected void waitOneCycle() {
                Timer.delay(1.0 / getUpdateFrequency());
            }
        };
    }

    private void setUpdateFrequency(double updateFrequency, StatusSignal<?> ... signals) {
        for (StatusSignal<?> signal : signals) {
            signal.setUpdateFrequency(updateFrequency, 1.0);
        }
    }

    private void setUpdatePeriod(double updateFrequency, TalonSRX motor, StatusFrameEnhanced ... frames) {
        int updatePeriodMillis = (int) Math.round(1000.0 / updateFrequency);

        for (StatusFrameEnhanced frame : frames) {
            motor.setStatusFramePeriod(frame, updatePeriodMillis, 1000);
        }
    }
}
