package botsnax.swerve.phoenix;

import botsnax.swerve.GenericSwerveDrivetrain;
import botsnax.swerve.SwerveCalibration;
import botsnax.swerve.SwerveController;
import botsnax.swerve.SwerveModule;
import botsnax.swerve.sim.GenericSwerveSim;
import botsnax.swerve.sim.IdealizedSwerveDrivetrain;
import botsnax.swerve.sim.IdealizedSwerveSim;
import botsnax.system.Gearbox;
import botsnax.system.Gyro;
import botsnax.system.encoder.phoenix.CANcoderEncoder;
import botsnax.system.motor.phoenix.PigeonGyro;
import botsnax.system.motor.phoenix.TalonFXMotor;
import botsnax.util.LinearAngularConversion;
import botsnax.util.SimRunner;
import botsnax.vision.PoseEstimate;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;

public class PhoenixSwerveDrivetrainFactory {
    protected final Mass mass;
    protected final SwerveDrivetrainConstants drivetrainConstants;
    protected final SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    protected PhoenixSwerveDrivetrainFactory(Mass mass, SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?> ... moduleConstants) {
        this.mass = mass;
        this.drivetrainConstants = drivetrainConstants;
        this.moduleConstants = moduleConstants;
    }

    public SwerveController create() {
        SwerveCalibration.apply(moduleConstants);

        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain = new SwerveDrivetrain<>(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                drivetrainConstants,
                moduleConstants
        );

        Pose2d initialPose = new Pose2d();
        DCMotor dcMotor = DCMotor.getKrakenX60(1);

        IdealizedSwerveDrivetrain idealizedSwerveDrivetrain = IdealizedSwerveDrivetrain.ofRectangularChassis(
                moduleConstants,
                dcMotor,
                mass);

        Gyro gyro = new PigeonGyro(drivetrain.getPigeon2());

        List<SwerveModule> genericModules = new ArrayList<>(moduleConstants.length);
        for (int i = 0; i < moduleConstants.length; i++) {
            SwerveModule genericModule = new SwerveModule(
                    new Gearbox(
                            new TalonFXMotor(
                                    drivetrain.getModule(i).getDriveMotor(),
                                    moduleConstants[i].DriveMotorInverted,
                                    dcMotor)
                                    .withGearRatio(moduleConstants[i].SteerMotorGearRatio),
                            new CANcoderEncoder(drivetrain.getModule(i).getEncoder())
                    ),
                    new TalonFXMotor(
                            drivetrain.getModule(i).getDriveMotor(),
                            moduleConstants[i].DriveMotorInverted,
                            dcMotor)
                            .withGearRatio(moduleConstants[i].DriveMotorGearRatio),
                    LinearAngularConversion.ofWheelRadiusGearRatio(
                            Meters.of(moduleConstants[i].WheelRadius),
                            moduleConstants[i].DriveMotorGearRatio
                    ));

            genericModules.add(genericModule);
        }

        GenericSwerveDrivetrain genericDrivetrain = new GenericSwerveDrivetrain(
                drivetrain.getKinematics(),
                genericModules.toArray(new SwerveModule[0])
        );

        IdealizedSwerveSim idealizedSwerveSim = idealizedSwerveDrivetrain.createSim(initialPose);
        GenericSwerveSim genericSwerveSim = new GenericSwerveSim(
                idealizedSwerveSim,
                gyro,
                genericModules.toArray(new SwerveModule[0]),
                drivetrain.getKinematics(),
                moduleConstants
        );

        if (isSimulation()) {
            new SimRunner().start(
                    dt -> genericSwerveSim.update(dt, Volts.of(getBatteryVoltage()), genericModules.toArray(new SwerveModule[0])),
                    Milliseconds.of(5));
        }

        return new SwerveController() {
            @Override
            public GenericSwerveDrivetrain getDrivetrain() {
                return genericDrivetrain;
            }

            @Override
            public Pose2d getPose() {
                return drivetrain.getState().Pose;
            }

            @Override
            public Pose2d getSimPose() {
                return idealizedSwerveSim.getPose();
            }

            @Override
            public void setPose(Pose2d pose) {
                drivetrain.resetPose(pose);
            }

            @Override
            public void setSimPose(Pose2d pose) {
                idealizedSwerveSim.setPose(pose);
            }

            @Override
            public void setFieldSpeeds(final Function<Pose2d, ChassisSpeeds> speeds) {
                drivetrain.setControl(new SwerveRequest.ApplyFieldSpeeds() {
                    @Override
                    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, com.ctre.phoenix6.swerve.SwerveModule<?, ?, ?>... modulesToApply) {
                        this.withSpeeds(speeds.apply(parameters.currentPose));
                        return super.apply(parameters, modulesToApply);
                    }
                });
            }

            @Override
            public void addVisionMeasurement(PoseEstimate estimate) {
                drivetrain.addVisionMeasurement(estimate.pose(), estimate.timestamp().in(Seconds), estimate.stdDevs());
            }
        };
    }
}
