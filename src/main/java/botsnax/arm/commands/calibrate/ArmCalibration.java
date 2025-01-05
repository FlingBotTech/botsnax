package botsnax.arm.commands.calibrate;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Preferences;
import botsnax.arm.commands.ArmKinematics;
import botsnax.commands.calibrate.VelocityCalibration;
import botsnax.control.MotorController;
import botsnax.control.ReactionTimeVelocityController;
import botsnax.control.SetpointController;
import botsnax.system.motor.MotorState;

import java.util.Optional;
import java.util.function.Function;

import static edu.wpi.first.units.ImmutableMeasure.ofRelativeUnits;
import static edu.wpi.first.units.Units.Degrees;
import static botsnax.control.SetpointController.ofVelocityController;
import static java.lang.Double.NaN;
import static java.lang.Math.signum;

public record ArmCalibration(
        ArmGravityController gravityController,
        double gearRatio,
        VelocityCalibration positiveVelocityCalibration,
        VelocityCalibration negativeVelocityCalibration) implements ArmKinematics {

    private static final Measure<Angle> DEADBAND = ofRelativeUnits(0.01, Degrees);

    @Override
    public double getGearRatio() {
        return gearRatio;
    }

    @Override
    public Measure<Angle> getHorizontalAngle() {
        return gravityController.offset();
    }

    @Override
    public double getVoltageForVelocity(Measure<Velocity<Angle>> velocity, MotorState state) {
        return getVoltage(velocity, state) + gravityController.calculate(state);
    }

    public double getVoltage(Measure<Velocity<Angle>> velocity, MotorState state) {
        double signOfVelocity = signum(velocity.baseUnitMagnitude());

        if (signOfVelocity > 0) {
            return positiveVelocityCalibration.getVoltageForVelocity(velocity, state);
        } else if (signOfVelocity < 0) {
            return negativeVelocityCalibration.getVoltageForVelocity(velocity, state);
        } else {
            return 0;
        }
    }

    public MotorController createController(Function<MotorState, Measure<Angle>> profile, Measure<Time> reactionTime, Measure<Velocity<Angle>> maxVelocity) {
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

    public Measure<Angle> getOffset() {
        return gravityController.offset();
    }

    public Measure<Angle> offsetByLevel(Measure<Angle> angle) {
        return angle.minus(gravityController.offset());
    }

    public Measure<Angle> getMotorAngleForEncoderAngle(Measure<Angle> encoderAngle) {
        return encoderAngle.times(gearRatio());
    }
}
