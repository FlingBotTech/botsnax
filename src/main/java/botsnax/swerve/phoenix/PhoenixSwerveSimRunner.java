package botsnax.swerve.phoenix;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import botsnax.swerve.sim.IdealizedSwerveSim;
import botsnax.swerve.sim.SwerveSim;

public class PhoenixSwerveSimRunner {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final IdealizedSwerveSim idealizedSwerveSim;

    public PhoenixSwerveSimRunner(IdealizedSwerveSim idealizedSwerveSim) {
        this.idealizedSwerveSim = idealizedSwerveSim;
    }

    public SwerveSim start(PhoenixSwerveDrivetrain drivetrain) {
        PhoenixSwerveSim sim = new PhoenixSwerveSim(
                idealizedSwerveSim,
                drivetrain.getModuleLocations(),
                drivetrain.getPigeon(),
                drivetrain.getDrivetrainConstants(),
                drivetrain.getModuleConstants());
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            sim.updateWithIdealized(deltaTime, RobotController.getBatteryVoltage(), drivetrain.getModules());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);

        return idealizedSwerveSim;
    }
}
