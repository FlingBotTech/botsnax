package botsnax.arm.commands;

import botsnax.arm.commands.calibrate.ArmCalibrationParams;
import botsnax.arm.commands.calibrate.CalibrateCommand;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.commands.MotorControllerCommand;
import botsnax.control.MotorController;
import botsnax.control.MotorVelocityController;
import botsnax.control.ReactionTimeVelocityController;
import botsnax.control.SetpointVelocityController;
import botsnax.system.motor.MotorSystem;
import botsnax.system.Gearbox;
import edu.wpi.first.units.measure.Angle;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ArmCommandFactory {
    private final ArmCalibrationParams calibrationParams;
    private final Gearbox gearbox;
    private final ArmKinematics kinematics;
    private final Subsystem subsystem;
    private final MotorSystem motor;
    private final Consumer<Angle> setpointObserver;

    public ArmCommandFactory(
            ArmCalibrationParams calibrationParams,
            Gearbox gearbox,
            ArmKinematics kinematics,
            Consumer<Angle> setpointObserver,
            Subsystem subsystem) {
        this.calibrationParams = calibrationParams;
        this.gearbox = gearbox;
        this.kinematics = kinematics;
        this.setpointObserver = setpointObserver;
        this.subsystem = subsystem;
        this.motor = gearbox.getMotor().withGearRatio(kinematics.getGearRatio());
    }

    public void initMotorPosition() {
        motor.setAngle(gearbox.getOutputEncoder().getAngle());
    }

    public Angle getAngle() {
        return motor.getAngle();
    }

    public AngularVelocity getVelocity() {
        return motor.getVelocity();
    }

    public Angle getAngleRelativeToLevel() {
        return getAngle().minus(kinematics.getHorizontalAngle());
    }

    public void setEncoderAngle(Angle angle) {
        gearbox.getOutputEncoder().setAngle(angle);
    }

    public Command calibrate() {
        return new CalibrateCommand(
                calibrationParams,
                Volts.of(12.0),
                calibration -> {
                    calibration.save(subsystem.getName());
                    System.out.println(calibration);
                    System.out.println("Arm is now calibrated, restart robot program to operate");
                },
                subsystem);
    }

    public Command moveUp(AngularVelocity velocity, Angle maxAngle) {
        return runController(state -> velocity)
                .until(() -> motor.getAngle().gte(maxAngle))
                .andThen(holdCurrentAngle());
    }

    public Command moveDown(AngularVelocity velocity) {
        return runController(state -> velocity)
                .until(() -> motor.getAngle().lte(Degrees.of(5)))
                .andThen(park());
    }

    public Command holdCurrentAngle() {
        return new InstantCommand(() -> {
            Angle angle = motor.getAngle();

            if (angle.gt(Degrees.of(5))) {
                goToAngle(() -> angle).schedule();
            } else {
                park().schedule();
            }
        }, subsystem);
    }

    public Command park() {
        return goToAngle(() -> Degrees.of(1))
                .until(() ->
                        motor.getAngle().lt(Degrees.of(1.5)) &&
                                motor.getVelocity().lt(Degrees.per(Second).of(0.1)))
                .andThen(new InstantCommand(motor::stop, subsystem));
    }

    public Command goToAngle(Supplier<Angle> angleSupplier) {
        return runController(SetpointVelocityController.ofProfile(
                        state -> {
                            Angle angle = angleSupplier.get();
                            setpointObserver.accept(angle);
                            return angle;
                        },
                        new ReactionTimeVelocityController(
                                Seconds.of(0.2),
                                Degrees.per(Seconds).of(120),
                                Degrees.of(0.01))
                )
        );
    }

    private Command runController(MotorVelocityController controller) {
        return new MotorControllerCommand(
                motor,
                MotorController.ofVelocityController(controller, kinematics),
                subsystem);
    }

    public Command goToAngleRelativeToLevel(Supplier<Angle> angleSupplier) {
        return goToAngle(() -> angleSupplier.get().plus(kinematics.getHorizontalAngle()));
    }
}
