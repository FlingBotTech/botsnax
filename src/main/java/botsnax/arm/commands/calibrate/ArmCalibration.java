package botsnax.arm.commands.calibrate;

import botsnax.arm.commands.ArmKinematics;
import botsnax.commands.calibrate.VelocityCalibration;
import botsnax.control.MotorController;
import botsnax.control.ReactionTimeVelocityController;
import botsnax.control.SetpointController;
import botsnax.system.motor.MotorState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

import java.util.Optional;
import java.util.function.Function;

import static botsnax.control.SetpointController.ofVelocityController;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static java.lang.Double.NaN;
import static java.lang.Math.signum;

public record ArmCalibration(
        ArmGravityController gravityController,
        double gearRatio,
        VelocityCalibration positiveVelocityCalibration,
        VelocityCalibration negativeVelocityCalibration) implements ArmKinematics {

    private static final Angle DEADBAND = Degrees.of(0.01);

    @Override
    public double getGearRatio() {
        return gearRatio;
    }

    @Override
    public Angle getHorizontalAngle() {
        return gravityController.offset();
    }

    @Override
    public Voltage getVoltageForVelocity(AngularVelocity velocity, MotorState state) {
        return getVoltage(velocity, state).plus(gravityController.calculate(state));
    }

    public Voltage getVoltage(AngularVelocity velocity, MotorState state) {
        double signOfVelocity = signum(velocity.baseUnitMagnitude());

        if (signOfVelocity > 0) {
            return positiveVelocityCalibration.getVoltageForVelocity(velocity, state);
        } else if (signOfVelocity < 0) {
            return negativeVelocityCalibration.getVoltageForVelocity(velocity, state);
        } else {
            return Volts.of(0);
        }
    }

    public MotorController createController(Function<MotorState, Angle> profile, Time reactionTime, AngularVelocity maxVelocity) {
        return SetpointController.ofProfile(profile, ofVelocityController(
                new ReactionTimeVelocityController(reactionTime, maxVelocity, DEADBAND),
                this
        ));
    }

    public ArmCalibration withGravity(ArmGravityController gravityCalibration) {
        return new ArmCalibration(gravityCalibration, gearRatio, positiveVelocityCalibration, negativeVelocityCalibration);
    }

    public ArmCalibration withGearRatio(double gearRatio) {
        return new ArmCalibration(gravityController, gearRatio, positiveVelocityCalibration, negativeVelocityCalibration);
    }

    public ArmCalibration withVelocity(VelocityCalibration positiveVelocityCalibration, VelocityCalibration negativeVelocityCalibration) {
        return new ArmCalibration(gravityController, gearRatio, positiveVelocityCalibration, negativeVelocityCalibration);
    }

    public void save(String baseName) {
        gravityController.save(baseName);
        Preferences.setDouble(baseName + ".gearRatio", gearRatio);
        positiveVelocityCalibration.save(baseName + ".positive");
        negativeVelocityCalibration.save(baseName + ".negative");
    }

    public static Optional<ArmCalibration> load(String baseName) {
        double gearRatio = Preferences.getDouble(baseName + ".gearRatio", NaN);

        if (!Double.isNaN(gearRatio)) {
            return ArmGravityController.load(baseName).flatMap(gravityCalibration ->
                    VelocityCalibration.load(baseName + ".positive").flatMap(positiveVelocityCalibration ->
                            VelocityCalibration.load(baseName + ".negative").map(negativeVelocityCalibration ->
                                    new ArmCalibration(gravityCalibration, gearRatio, positiveVelocityCalibration, negativeVelocityCalibration))));
        } else {
            return Optional.empty();
        }
    }

    public Angle getOffset() {
        return gravityController.offset();
    }

    public Angle offsetByLevel(Angle angle) {
        return angle.minus(gravityController.offset());
    }

    public Angle getMotorAngleForEncoderAngle(Angle encoderAngle) {
        return encoderAngle.times(gearRatio());
    }
}
