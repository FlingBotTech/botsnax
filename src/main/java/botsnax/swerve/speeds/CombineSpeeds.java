package botsnax.swerve.speeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;

public class CombineSpeeds {
    public static SpeedsFunction plus(SpeedsFunction f1, SpeedsFunction f2) {
        return pose -> f1.apply(pose).plus(f2.apply(pose));
    }

    public static SpeedsFunction concatAndStop(SpeedsFunction ... functions) {
        final ChassisSpeeds STOP = new ChassisSpeeds(0, 0, 0);

        return new SpeedsFunction() {
            int currentIndex = 0;

            @Override
            public ChassisSpeeds apply(Pose2d pose) {
                while (currentIndex < functions.length) {
                    ChassisSpeeds currentSpeeds = functions[currentIndex].apply(pose);

                    if (currentSpeeds.equals(STOP)) {
                        currentIndex++;
                    } else {
                        return currentSpeeds;
                    }
                }

                return STOP;
            }
        };
    }
}
