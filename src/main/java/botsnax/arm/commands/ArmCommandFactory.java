package botsnax.arm.commands;

import botsnax.arm.commands.calibrate.ArmCalibrationParams;
import botsnax.arm.commands.calibrate.CalibrateCommand;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
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

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ArmCommandFactory {
    private final ArmCalibrationParams calibrationParams;
    private final Gearbox gearbox;
    private final ArmKinematics kinematics;
    private final Subsystem subsystem;
    private final MotorSystem motor;
    private final Consumer<Measure<Angle>> setpointObserver;

    public ArmCommandFactory(
            ArmCalibrationParams calibrationParams,
            Gearbox gearbox,
            ArmKinematics kinematics,
            Consumer<Measure<Angle>> setpointObserver,
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

    public Measure<Angle> getAngle() {
        return motor.getAngle();
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return motor.getVelocity();
    }

    public Measure<Angle> getAngleRelativeToLevel() {
        return getAngle().minus(kinematics.getHorizontalAngle());
    }

    public void setEncoderAngle(Measure<Angle> angle) {
        gearbox.getOutputEncoder().setAngle(angle);
    }

    public Command calibrate() {
        return new CalibrateCommand(
                calibrationParams,
                12.0,
                calibration -> {
                    calibration.save(subsystem.getName());
                    System.out.println(calibration);
                    System.out.println("Arm is now calibrated, restart robot program to operate");
                },
                subsystem);
    }

    public Command moveUp(Measure<Velocity<Angle>> velocity, Measure<Angle> maxAngle) {
        return runController(state -> velocity)
                .until(() -> motor.getAngle().gte(maxAngle))
                .andThen(holdCurrentAngle());
    }

    public Command moveDown(Measure<Velocity<Angle>> velocity) {
        return runController(state -> velocity)
                .until(() -> motor.getAngle().lte(Degrees.of(5)))
                .andThen(park());
    }

    public Command holdCurrentAngle() {
        return new InstantCommand(() -> {
            Measure<Angle> angle = motor.getAngle();

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

    public Command goToAngle(Supplier<Measure<Angle>> angleSupplier) {
        return runController(SetpointVelocityController.ofProfile(
                        state -> {
                            Measure<Angle> angle = angleSupplier.get();
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

    public Command goToAngleRelativeToLevel(Supplier<Measure<Angle>> angleSupplier) {
        return goToAngle(() -> angleSupplier.get().plus(kinematics.getHorizontalAngle()));
    }
}
